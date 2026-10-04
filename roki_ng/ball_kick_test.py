"""One-shot ball approach and jump alignment, independent of field strategy."""
import math

from .calibration import wrap
from .wire import Fault


def kick_alignment(p, x, y, side_target):
    """Shared ball-test alignment decision; coordinates and target are millimetres."""
    front, side = x-p['game.kick_distance_mm'], y-side_target
    ready = front <= 10 and abs(side) <= 20 and math.hypot(x, y) < 200
    direction, error = (('left' if side > 0 else 'right'), abs(side)) if abs(side) > 20 else ('forward', front)
    fraction = min(1., max(.1, error/p[f'motion.jump_{direction}_mm']))
    return ready, direction, fraction, front, side


async def run_ball_kick_test(game, p, entry):
    revision = game.revision

    async def observe():
        result = await game._original_observe(p, entry, settle=.5)
        if not game._same_plan(revision):
            raise Fault('cancelled', 'Ball kick test interrupted')
        ball = game._usable(result)
        if ball is None:
            raise Fault('no_ball', 'No fresh ball in support pose; place ball in front and restart test')
        game.s.log('game', 'INFO', f"Ball kick test: x={ball['x_m']*1000:.0f} y={ball['y_m']*1000:.0f} mm")
        return ball

    # Only the far approach may walk. Once fine alignment begins, never fall
    # back to verify/orbit walking or the field-dependent kick table.
    game.info.update(state='searching', phase='ball_test_observe')
    ball = await observe()
    for approach in range(4):
        distance = math.hypot(ball['x_m'], ball['y_m'])
        if distance <= .35:
            break
        if approach == 3:
            raise Fault('approach_limit', 'Ball kick test: three approaches did not reach 350 mm')
        game.info.update(state='approaching', phase='ball_test_approach')
        heading = wrap(game.odometry.yaw + math.atan2(ball['y_m'], ball['x_m']))
        if not await game._original_turn(heading, for_approach=True):
            raise Fault('turn_failed', 'Ball kick test: could not face ball')
        if not game._same_plan(revision):
            return False
        ball = await observe()
        distance = math.hypot(ball['x_m'], ball['y_m'])
        if distance > .35:
            if not await game._original_walk(p, entry, 'approach', ball, distance_m=distance,
                                            heading=wrap(heading-game.odometry.home_heading)):
                return False
            ball = await observe()

    # Choose one leg and retain it throughout alignment, including Y=0.
    side_target = math.copysign(p['game.kick_offset_mm'], ball['y_m'])
    game.s.log('game', 'INFO', f"Ball kick test: fine alignment, leg={'left' if side_target > 0 else 'right'}; no more walking")
    for jumps in range(6):
        game.info.update(state='approaching', phase='ball_test_fine_adjustment')
        x, y = ball['x_m']*1000, ball['y_m']*1000
        if math.hypot(x, y) > 350:
            raise Fault('ball_moved', 'Ball left the 350 mm alignment zone; restart test')
        ready, direction, fraction, front, side = kick_alignment(p, x, y, side_target)
        # Tighter lateral acceptance than the match's emergency envelope:
        # this test must actually align to the chosen foot, not kick at Y=0.
        if ready:
            await game._imu()
            if not game._same_plan(revision):
                return False
            return await game._original_kick(revision, game.odometry.yaw, 100, ball, x, y,
                                             reason='ball kick test; no goal selection')
        if jumps == 5:
            raise Fault('fine_adjustment_limit', 'Five jumps completed without alignment; test finished without walking or kicking')
        # One jump per fresh observation; correct lateral placement first.
        game.s.log('game', 'INFO', f'Ball kick test: jump {jumps+1}/5 {direction}, fraction={fraction:.2f}, front={front:.0f} side={side:.0f} mm')
        if not await game._move('game.jump', {'direction': direction, 'fraction': fraction}):
            return False
        ball = await observe()
    return False
