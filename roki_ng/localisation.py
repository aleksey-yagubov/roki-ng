"""Bounded visual pose candidates. No motion commands; no certified body pose yet.

Runtime outputs remain invalid until camera/body extrinsics and metric accuracy
have independent validation. Unique sequences are required within one capture.
"""
import math
import time

import numpy as np

from .field_observations import field_model, update


class PoseFilter:
    def __init__(self, prior, count=2048, seed=27, parameters=None):
        prior = np.asarray(prior, dtype=float)
        if prior.shape != (3,) or not np.isfinite(prior).all():
            raise ValueError('Invalid pose prior')
        if type(count) is not int or not 256 <= count <= 8192:
            raise ValueError('Particle count must be 256..8192')
        geometry=parameters.get('field.geometry') if parameters else None
        bounds=(geometry['carpet_length']/2+1,geometry['carpet_width']/2+1) if geometry else (3,3)
        if abs(prior[0]) > bounds[0] or abs(prior[1]) > bounds[1] or abs(prior[2]) > math.pi:
            raise ValueError('Pose prior outside supported range')
        self.rng = np.random.default_rng(seed)
        self.prior = prior.copy()
        self.particles = self.rng.normal(prior, [.55, .55, .44], (count, 3))
        self.particles[:, 2] = (self.particles[:, 2]+np.pi) % (2*np.pi)-np.pi
        self.weights = np.full(count, 1/count)
        self.sequence = -1
        self.anchor = None
        self.anchor_time = None
        self.goal_foot_tolerance = (parameters or {}).get('localisation.goal_foot_tolerance_m',.35)
        self.max_speed = (parameters or {}).get('localisation.max_speed_m_s', .5)
        self.max_turn = (parameters or {}).get('localisation.max_turn_rad_s', 2.)
        from .field_config import line_model, circle_model
        self.model = line_model(parameters) if parameters else field_model()
        self.circles = circle_model(parameters) if parameters else None
        self.goals = [parameters[f'field.goal.{i}'] for i in range(2)] if parameters else []
        self.own_goal = parameters.get('match.own_goal',0) if parameters else 0

    def update(self, sequence, lines, circle, goals=None, *, timestamp=None):
        if not isinstance(sequence, int) or sequence <= self.sequence:
            raise ValueError('Repeated or out-of-order frame sequence')
        now = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(now) or (self.anchor_time is not None and now < self.anchor_time):
            raise ValueError('Nonmonotonic observation time')
        self.sequence = sequence
        if len(lines) < 3:
            return {'valid': False, 'candidate': None, 'reason': 'insufficient_observations',
                    'frame_sequence': sequence, 'lines': len(lines)}
        # Per-frame measurement weights, not posterior multiplication of
        # correlated stationary views. Resampling guides the next proposal only.
        candidates = self.particles
        # A wrong translation prior must not prevent a visible circle from
        # proposing poses. Keep all four field-axis orientations: a symmetric
        # field cannot establish the side from white paint alone.
        if circle is not None and (self.circles is None or len(self.circles) == 1):
            centre = np.asarray(circle['center_robot_m'], dtype=float)
            landmark = np.zeros(2) if self.circles is None else np.asarray(self.circles[0]['center'])
            segments = np.asarray(lines)
            vectors = segments[:, 1] - segments[:, 0]
            direction = vectors[np.argmax(np.linalg.norm(vectors, axis=1))]
            base = -math.atan2(direction[1], direction[0])
            n = len(self.particles)//2
            yaw = base + (np.arange(n)%4)*math.pi/2
            recovery = np.column_stack((landmark[0]-np.cos(yaw)*centre[0]+np.sin(yaw)*centre[1],
                                        landmark[1]-np.sin(yaw)*centre[0]-np.cos(yaw)*centre[1], yaw))
            # Use a frame-independent cloud; repeated static observations must
            # not randomly remove one of the symmetric hypotheses.
            recovery += np.random.default_rng(41).normal(0, [.08, .08, .05], recovery.shape)
            recovery[:, 2] = (recovery[:, 2]+math.pi)%(2*math.pi)-math.pi
            candidates = np.concatenate((self.particles[:len(self.particles)-n], recovery))
        weights, errors = update(candidates, lines, self.model, circle, self.circles)
        from .goal_observations import bearing_log_likelihood
        goal_log, goal_pairs = bearing_log_likelihood(candidates, goals or [], self.goals, self.goal_foot_tolerance)
        if goal_pairs:
            weights *= np.exp(goal_log-goal_log.max())
            weights /= weights.sum()
        best = int(weights.argmax())
        pose = candidates[best].copy()
        result = {'valid': False, 'candidate': pose.tolist(),
                  'reason': 'unverified_camera_body_calibration',
                  'frame_sequence': sequence, 'lines': len(lines),
                  'own_goal': self.own_goal, 'goal_pairs': goal_pairs,
                  'circle': circle is not None,
                  'median_residual_m': float(np.median(errors[best])),
                  'inlier_fraction': float(np.mean(errors[best] < .10)),
                  'ess': float(1/(weights@weights))}
        if result['inlier_fraction'] < .6 or result['median_residual_m'] > .10:
            result['reason'] = 'unverified_camera_body_calibration; weak_geometry'
        segments=np.asarray(lines,dtype=float)
        vectors=segments[:,1]-segments[:,0]
        directions=np.arctan2(vectors[:,1],vectors[:,0])
        supported=errors[best]<.10
        angles=directions[supported]
        independent=bool(len(angles)>1 and np.max(np.abs(np.sin(angles[:,None]-angles)))>.5)
        # A parallel edge family cannot constrain translation along the lines.
        strong=result['inlier_fraction']>=.6 and result['median_residual_m']<=.10
        result['fit_state']='matched' if strong and (independent or circle is not None) else 'weak'
        delta=candidates-pose
        delta[:,2]=(delta[:,2]+np.pi)%(2*np.pi)-np.pi
        local=(np.linalg.norm(delta[:,:2],axis=1)<.35)&(np.abs(delta[:,2])<.35)
        mass=float(weights[local].sum())
        result['mode_mass']=mass
        result['ambiguous']=mass<.6
        if result['ambiguous']:result['fit_state']='ambiguous'
        # Proposal spread is not a calibrated physical error bar.
        result['proposal_spread']=np.sqrt(np.sum(weights[:,None]*delta**2,axis=0)).tolist()
        # Do not turn missing information into confidence by resampling a
        # weak view or randomly discarding one of two symmetric modes.
        if result['fit_state'] != 'matched':
            return result
        if self.anchor is not None:
            elapsed = now-self.anchor_time
            translation = float(np.linalg.norm(pose[:2]-self.anchor[:2]))
            turn = abs((pose[2]-self.anchor[2]+math.pi)%(2*math.pi)-math.pi)
            if translation > .15+self.max_speed*elapsed or turn > .15+self.max_turn*elapsed:
                result.update(candidate=None, fit_state='rejected', reason='motion_discontinuity')
                return result
        self.anchor, self.anchor_time = pose.copy(), now
        self.weights = weights
        # Systematic proposal resampling with a broad recovery component.
        n = len(weights)
        indexes = np.searchsorted(np.cumsum(weights), (self.rng.random()+np.arange(n))/n)
        indexes = np.minimum(indexes, n-1)
        proposal = candidates[indexes]+self.rng.normal(0, [.05, .05, .04], (n, 3))
        recover = n//5
        proposal[:recover] = self.rng.normal(self.prior, [.65, .65, .5], (recover, 3))
        proposal[:, 2] = (proposal[:, 2]+np.pi) % (2*np.pi)-np.pi
        self.particles = proposal
        self.weights = np.full(n, 1/n)
        return result
