#!/usr/bin/env python3

import subprocess
import sys

ENTITY_NAME = 'ackermann_drive'
X, Y, Z = 120.0, 32.0, 0.6
WORLD_NAME = 'default'


def teleport():
    service = f'/world/{WORLD_NAME}/set_entity_pose'

    req = (
        f'name: "{ENTITY_NAME}" '
        f'position: {{x: {X}, y: {Y}, z: {Z}}} '
        f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
    )

    cmd = [
        'gz', 'service',
        '-s', service,
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ]

    print(f'Calling: {" ".join(cmd)}')
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0:
        print(f'Success: {result.stdout.strip()}')
    else:
        print(f'Failed: {result.stderr.strip()}')
        sys.exit(1)


if __name__ == '__main__':
    teleport()