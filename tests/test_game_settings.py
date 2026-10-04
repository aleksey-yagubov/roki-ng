import asyncio
from unittest.mock import AsyncMock
from types import SimpleNamespace

import pytest

from roki_ng.supervisor import Supervisor
from roki_ng.parameters import Parameters
from roki_ng.head_menu import HeadMenu, menu_tree
from roki_ng.wire import Fault


def test_voice_selects_goal_by_current_map_colour_not_fixed_id(tmp_path):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        s.mode = 'IDLE'
        s.params.values['field.goal.0']['colour'] = 'blue'
        s.params.values['field.goal.1']['colour'] = 'yellow'
        s.workers['motherboard'] = SimpleNamespace(alive=True, call=AsyncMock(return_value={}))
        result = await s._head_command('menu.own_goal', {'colour': 'yellow'})
        assert result['own_goal'] == 1
        assert Parameters(tmp_path).values['match.own_goal'] == 1
        assert s.owner is None and not s.game.info['running']
        s.workers['motherboard'].call.assert_not_called()
        s.params.values['field.goal.0']['colour'] = 'yellow'
        with pytest.raises(Fault):
            await s._head_command('menu.own_goal', {'colour': 'yellow'})
    asyncio.run(run())


def test_voice_goal_selection_never_acquires_or_revokes_gui_control(tmp_path):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        s.owner, s.lease_epoch = 99, 4
        s.workers['motherboard'] = SimpleNamespace(alive=True, call=AsyncMock(return_value={}))
        with pytest.raises(Fault) as error:
            await s._head_command('menu.own_goal', {'colour': 'yellow'})
        assert error.value.code == 'busy'
        assert s.owner == 99 and s.lease_epoch == 4
        s.workers['motherboard'].call.assert_not_called()
    asyncio.run(run())


def test_goal_menu_setting_does_not_create_or_clear_game_job():
    async def run():
        spoken = []
        command = AsyncMock(return_value={'colour': 'blue', 'own_goal': 1})
        menu = HeadMenu(command, AsyncMock(), spoken.append, lambda *a: None)
        menu.game_active = menu.pickup_mode = True
        menu.job = 'existing-game'
        item = menu_tree().children[0].children[0].children[2].children[1]
        await menu._start(item)
        assert menu.job == 'existing-game' and menu.game_active and menu.pickup_mode
        assert spoken[-1] == 'Own blue goal selected'
    asyncio.run(run())


def test_disabled_visual_localisation_does_not_open_worker_or_read_imu(tmp_path):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        s.game._call = AsyncMock()
        s.game._imu = AsyncMock()
        s.workers['localisation'] = SimpleNamespace(alive=True)
        s.game.odometry = SimpleNamespace(x=0., y=0., yaw=0.)
        s.game.needs_prepare = False
        s.game.head_pan = 0
        p = s.params.values | {'game.use_visual_localisation': False}
        await s.game._open_localisation(p)
        s.game._call.assert_not_called()
        s.game._imu.assert_not_called()
        s.params.set('game.use_visual_localisation', False)
        assert Parameters(tmp_path).values['game.use_visual_localisation'] is False
    asyncio.run(run())
