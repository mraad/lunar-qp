"""Generate the QP app's recordings and build its static files. Run with uv."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

if __name__ == '__main__':
    for prefix, seed, episodes in (('', 0, 8), ('heldout-', 300, 20)):
        for scenario in ('nominal', 'fault'):
            for controller in ('adaptive', 'fixed'):
                name = f'{prefix}{scenario}-{controller}'
                flags = ['--fixed-model'] if controller == 'fixed' else []
                subprocess.run([sys.executable, '-m', 'lunar_qp.qp', '--seed', str(seed),
                                '--episodes', str(episodes), '--thrust-scale',
                                '.7' if scenario == 'fault' else '1', *flags,
                                '--out', f'dist/qp/{name}.json'], cwd=ROOT, check=True)
    subprocess.run([sys.executable, 'web/build.py'], cwd=ROOT, check=True)
