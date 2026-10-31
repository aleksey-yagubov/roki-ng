"""Shared scaling of original finite jump rows."""
import copy

def jump_rows(jumps, direction, fraction):
    if direction.startswith('turn_'):
        rows = [[10] + [0] * 21, [2] + [0] * 21, [2] + [0] * 21]
        rows[1][1], rows[1][12] = (-700, 700)
        rows[1][6] = rows[1][17] = round((1000 if direction == 'turn_right' else -1000) * fraction)
        return rows
    rows = copy.deepcopy(jumps[direction])
    if direction in ('forward', 'backward'):
        for row in rows[:2]:
            for index in (2, 13):
                row[index] = round(row[index] * fraction)
    else:
        value = round((200 if direction == 'right' else -200) * fraction)
        for index in (1, 5, 12, 16):
            rows[0][index] = value
        rows[1][5] = rows[1][16] = value
    return rows
