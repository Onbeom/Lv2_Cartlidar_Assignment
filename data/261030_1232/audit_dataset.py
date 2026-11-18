"""실물 라벨 확인 상태·원본 존재·클래스·좌표·중복 감사. 원본은 읽기만 한다."""
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path('/home/pa4/git/lv2_next_module')
OUT = ROOT / 'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1232'
NAMES4 = ['red_cylinder', 'green_cylinder', 'red_cuboid', 'green_cuboid']
NAMES6 = ['red_cylinder', 'green_cylinder', 'blue_cylinder', 'red_cuboid', 'green_cuboid', 'blue_cuboid']
REMAP = {0: 0, 1: 1, 3: 2, 4: 3}
groups, errors, hashes = [], [], defaultdict(list)
for dirname in ['real_labels', 'real_labels_261027', 'real_labels_261027_far_dark']:
    base = ROOT / '261026_1414' / dirname
    names = (base / 'classes.txt').read_text().splitlines()
    names = [n.strip() for n in names if n.strip()]
    if names not in (NAMES4, NAMES6):
        errors.append({'group': dirname, 'error': 'unknown_class_names', 'names': names})
    rows = json.loads((base / 'auto_label.json').read_text())
    batches, classes = defaultdict(Counter), Counter()
    positives = negatives = checked = 0
    for row in rows:
        if not row.get('checked'):
            continue
        checked += 1
        image = base / 'images' / row['image']
        label = base / 'labels' / (image.stem + '.txt')
        batch = row['batch'].split('_')[0]
        batches[batch]['images'] += 1
        if not image.is_file() or not label.is_file():
            errors.append({'image': str(image), 'error': 'missing_image_or_label'})
            continue
        lines = [ln.split() for ln in label.read_text().splitlines() if ln.strip()]
        positives += bool(lines)
        negatives += not bool(lines)
        for line in lines:
            try:
                cid = int(line[0])
                xywh = list(map(float, line[1:]))
                assert len(line) == 5 and len(xywh) == 4
                assert 0 <= cid < len(names)
                assert all(0 <= v <= 1 for v in xywh)
                assert xywh[2] > 0 and xywh[3] > 0
                mapped = REMAP[cid] if names == NAMES6 else cid
                classes[mapped] += 1
                batches[batch][NAMES4[mapped]] += 1
            except (ValueError, AssertionError, KeyError):
                errors.append({'label': str(label), 'error': 'invalid_or_excluded_label', 'line': line})
        digest = hashlib.sha256(image.read_bytes()).hexdigest()
        hashes[digest].append({'group': dirname, 'batch': batch, 'image': image.name,
                               'label_sha256': hashlib.sha256(label.read_bytes()).hexdigest()})
    groups.append({'group': dirname, 'total': len(rows), 'checked': checked,
                   'excluded_unchecked': len(rows) - checked, 'positive_images': positives,
                   'negative_images': negatives, 'instances_4class': dict(classes),
                   'batches': {b: dict(v) for b, v in sorted(batches.items())}})
keep_path = ROOT / '261026_1414/data_ft1/split.json'
keep = json.loads(keep_path.read_text())['setting']['split']
duplicates = [{'sha256': h, 'items': items} for h, items in hashes.items() if len(items) > 1]
cross_batch = [d for d in duplicates if len({i['batch'] for i in d['items']}) > 1]
red_batches = sorted({b for g in groups for b, counts in g['batches'].items()
                      if counts.get('red_cylinder', 0) + counts.get('red_cuboid', 0) > 0})
report = {'kind': 'dataset_audit_not_accuracy_evaluation', 'groups': groups,
          'checked_total': sum(g['checked'] for g in groups),
          'red_positive_batches': red_batches, 'ft1_fixed_split': keep,
          'duplicate_content': duplicates, 'cross_batch_duplicate_groups': len(cross_batch),
          'errors': errors, 'integrity_ok': not errors,
          'real_three_way_red_split_feasible': len(red_batches) >= 3,
          'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
OUT.mkdir(parents=True, exist_ok=True)
target = OUT / 'dataset_audit.json'
if target.exists():
    raise SystemExit('기존 결과를 덮어쓰지 않습니다')
target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({k: report[k] for k in ['checked_total', 'red_positive_batches',
      'cross_batch_duplicate_groups', 'integrity_ok', 'real_three_way_red_split_feasible']}, ensure_ascii=False))
