"""Compact the recorded QP experiments into a dependency-free static app."""
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'dist/web'


def rounded(values):
    return [round(float(value), 4) for value in values]


def compact(run, group):
    if run['config']['horizon'] != 16 or run['config']['hold'] != 4:
        raise ValueError('The presentation expects 16 four-step prediction blocks')
    episodes = []
    for episode in run['episodes']:
        frames = []
        for record in episode['records']:
            qp = record['qp']
            fractions = qp.get('probabilities', [1., 0., 0., 0.])
            assert len(fractions) == 4 and abs(sum(fractions)-1) < 1e-6
            assert len(record['state']) == 8 and record['action'] in range(4)
            frames.append({
                's': rounded(record['state']), 'a': record['action'],
                'p': [rounded(point[:2]) for point in record['prediction']],
                'gain': round(record['model_before'][0], 4), 'nextGain': round(record['model'][0], 4),
                'ms': round(record['solve_ms'], 3), 'fractions': fractions,
                'status': qp['status'], 'iterations': qp['iterations'],
                'slack': qp['slack_max'], 'violation': qp['constraint_violation'],
                'fallback': qp['fallback'],
            })
        assert frames and len(frames) == episode['steps'] and len(episode['final_state']) == 8
        episodes.append({
            'seed': episode['seed'], 'group': group, 'start': episode['start'],
            'terrain': episode['terrain'], 'pad': episode['helipad'],
            'passed': episode['passed'], 'margin': episode['pad_margin'],
            'final': episode['final_state'], 'frames': frames,
            'sourceSha256': run['source_sha256'],
        })
    return episodes


def main():
    datasets = {}
    page = (ROOT / 'web/index.html').read_text()
    for scenario in ('nominal', 'fault'):
        for controller in ('adaptive', 'fixed'):
            name = f'{scenario}-{controller}'
            episodes = []
            for prefix, group in (('', 'Development'), ('heldout-', 'Evaluation')):
                run = json.loads((ROOT / 'dist/qp' / f'{prefix}{name}.json').read_text())
                assert run['thrust_scale'] == (.7 if scenario == 'fault' else 1)
                assert run['change_step'] == 100
                assert run['config']['adaptive'] == (controller == 'adaptive')
                episodes.extend(compact(run, group))
            assert len({e['seed'] for e in episodes}) == len(episodes)
            evaluation = [e for e in episodes if e['group'] == 'Evaluation']
            result = f'{sum(e["passed"] for e in evaluation)} / {len(evaluation)}'
            page = page.replace('__RESULT_'+name.upper().replace('-', '_')+'__', result)
            datasets[name] = json.dumps({'changeStep': 100, 'thrustScale': run['thrust_scale'],
                                         'episodes': episodes}, separators=(',', ':'), allow_nan=False)
    # Validate every input before replacing any built assets.
    (OUTPUT / 'data').mkdir(parents=True, exist_ok=True)
    (OUTPUT / 'index.html').write_text(page)
    for name in ('styles.css', 'app.js'):
        shutil.copyfile(ROOT / 'web' / name, OUTPUT / name)
    for name, payload in datasets.items():
        (OUTPUT / 'data' / f'{name}.json').write_text(payload + '\n')
        print(f'{name}: {len(json.loads(payload)["episodes"])} flights, {len(payload)/1024:.0f} KB')
    print(f'Built {OUTPUT}')


if __name__ == '__main__':
    main()
