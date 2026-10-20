#!/usr/bin/env python3
"""Validate all reruns, preserve old plots, and publish consistent accuracy plots."""
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np
import plot_offline_errors as plots

BASE = Path(__file__).resolve().parent / 'logs/zab-offline'
NEW = BASE / 'vio_recomputed'
OLD = BASE / 'results'
OUT = BASE / 'error_plots'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(data):
    e = data['vio'][1]
    distance = np.linalg.norm(e[:, :3], axis=1)
    return dict(position_rmse_m=float(np.sqrt(np.mean(distance**2))),
                position_max_m=float(distance.max()),
                yaw_max_deg=float(np.abs(e[:, 5]).max()))


def main():
    manifest = json.loads((NEW / 'manifest.json').read_text())
    original = json.loads((OLD / 'manifest.json').read_text())
    assert manifest['status'] == 'complete'
    assert [r['label'] for r in manifest['runs']] == [r['label'] for r in original['runs']]
    manifest['runs'] = [json.loads((NEW / r['label'] / 'validation.json').read_text()) for r in original['runs']]
    assert len(manifest['runs']) == 14
    manifest['software'] = {
        'filter_binary_sha256': sha(BASE / 'vio_audit/audit25'),
        'replay_script_sha256': sha(Path(__file__).with_name('reprocess_vio_accuracy.py')),
        'reused_experiments': ['run1_tail_a2_prefix', 'run1_tail_b2', 'run2a_end'],
    }
    (NEW / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    validation, comparison = {}, {}
    reference_config = None
    for run in manifest['runs']:
        label = run['label']
        dest = NEW / label
        config = (dest / 'rovio.info').read_text()
        if reference_config is None:
            reference_config = config
        assert config == reference_config, 'Settings must be identical across all flights'
        for name in ('truth.csv', 'lio.csv', 'window.json'):
            assert sha(dest / name) == sha(OLD / label / name) == run['hashes'][name]
        assert sha(dest / 'vio.csv') == run['hashes']['vio.csv']
        old_t, old_p, old_q = plots.read(OLD / label / 'vio.csv')
        t, p, q = plots.read(dest / 'vio.csv')
        assert np.array_equal(t, old_t)
        assert np.isfinite(p).all()
        log = (dest / 'replay.log').read_text()
        original_log = (BASE / label / 'rovio.log').read_text()
        original_init = re.findall(r'-- Filter: Initialized at t = ([0-9.]+)', original_log)
        assert len(original_init) == 1
        expected_init_ns = round(float(original_init[0]) * 1e9)
        initialized = re.findall(r'-- Filter: Initialized at t = ([0-9.]+)', log)
        assert len(initialized) == 1 and round(float(initialized[0]) * 1e9) == expected_init_ns
        assert 'addPredictionMeas] Warning' not in log
        late = [line for line in log.splitlines() if 'addUpdateMeas] Warning' in line]
        assert all(f'time 2 before safeTime {initialized[0]}' in line for line in late), (label, late)
        delivered = re.search(r'AUDIT_INPUT (\d+) (\d+) (\d+) (\d+) sets (\d+) incomplete (\d+)', log)
        info = json.loads((dest / 'run.json').read_text())
        assert delivered and list(map(int, delivered.groups()[:4])) == info['counts']
        if 'tracking_camera_stamps' in info:
            missing = sorted(set(info['tracking_camera_stamps']) - set(t))
            expected = [1323150000000, 1323200000000] if label == 'run1_middle' else []
            assert missing == expected, (label, missing)
            assert set(t).issubset(set(info['tracking_camera_stamps']))
        new_data, new_meta = plots.errors(dest)
        old_data, old_meta = plots.errors(OLD / label)
        assert new_meta['anchor_ns'] == old_meta['anchor_ns']
        assert np.array_equal(new_data['lio'][1], old_data['lio'][1])
        assert np.array_equal(new_data['vio'][0], old_data['vio'][0])
        assert np.max(np.abs(new_data['vio'][1][0])) < 1e-8
        comparison[label] = dict(old=metrics(old_data), new=metrics(new_data),
                                 evaluated_samples=len(new_data['vio'][0]))
        validation[label] = dict(count=len(t), exact_old_timestamps=True,
                                 unchanged_truth_and_lio=True, initialization_count=1,
                                 initialization_stamp_ns=expected_init_ns,
                                 late_imu_count=0, evaluated_samples=len(new_data['vio'][0]))
    (NEW / 'comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
    archive = BASE / 'error_plots_before_vio_recompute'
    if not archive.exists():
        shutil.copytree(OUT, archive)
    # Generate into a separate directory, validate, then replace individual artifacts.
    stage = BASE / 'error_plots_vio_recomputed'
    subprocess.run([sys.executable, str(Path(plots.__file__)), '--source', str(NEW),
                    '--output', str(stage)], check=True)
    evaluation = json.loads((stage / 'evaluation.json').read_text())
    for label, meta in evaluation.items():
        for k in ('lio', 'vio'):
            with (stage / (label + '_' + k + '_errors.csv')).open() as stream:
                rows = list(csv.DictReader(stream))
            ts = np.array([int(r['stamp_ns']) for r in rows], dtype=np.int64)
            expected, _ = plots.errors(NEW / label)
            assert np.array_equal(ts, expected[k][0])
            values = np.array([[float(r[n]) for n in ('ex_m', 'ey_m', 'ez_m', 'eroll_deg', 'epitch_deg', 'eyaw_deg')] for r in rows])
            assert np.array_equal(values, expected[k][1])
            if k == 'lio':
                assert (stage / (label + '_lio_errors.csv')).read_bytes() == (archive / (label + '_lio_errors.csv')).read_bytes()
        for value in meta['display'].values():
            assert sum(value['omitted_sample_counts'].values()) == 0
    for link in re.findall(r'(?:src|href)="([^"]+)"', (stage / 'index.html').read_text()):
        if not link.startswith('#'):
            assert (stage / link).exists(), link
    pdf = (stage / 'all_flights.pdf').read_bytes()
    assert pdf.startswith(b'%PDF-') and pdf.rstrip().endswith(b'%%EOF')
    assert len(re.findall(rb'/Type /Page\b', pdf)) == 14
    total = sum(v['count'] for v in validation.values())
    kept = sum(v['evaluated_samples'] for v in validation.values())
    result = dict(status='passed', runs=validation, output_count=total,
                  evaluated_count=kept, old_plots_preserved=str(archive),
                  same_lio_error_csv_bytes=True, original_ns_preserved=True,
                  no_axis_breaks=True, pdf_pages=14)
    (NEW / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    (stage / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    shutil.copy2(NEW / 'comparison.json', stage / 'comparison.json')
    for path in stage.iterdir():
        shutil.copy2(path, OUT / path.name)
    # Keep estimator internals and the initialization period outside the accuracy dataset.
    diagnostics = BASE / 'vio_audit/recomputed_diagnostics'
    for run in manifest['runs']:
        label = run['label']
        for name in ('result_vio.csv', 'result_state.csv', 'result_features.csv'):
            source = NEW / label / name
            if source.exists():
                dest = diagnostics / label / name
                dest.parent.mkdir(parents=True, exist_ok=True)
                if dest.exists():
                    assert sha(source) == sha(dest)
                    source.unlink()
                else:
                    source.rename(dest)
    (NEW / 'README.txt').write_text('''全14飛行の再計算VIO精度データ（2026-10-03）
各飛行のvio.csv: ROVIOの元座標系での位置・姿勢・速度。stamp_nsは元センサーの整数ns。
truth.csv / lio.csv: 旧resultsから内容を変更せずコピー。window.json: 軌道追従期間。
window.jsonのrate_capは旧処理の記録。今回の直接入力は実時間への速度制限なしで実行。
全14飛行で同じgyro_matched_depth10設定。真値を推定器へ与えていない。
再計算11飛行＋前回同設定で検証済みの3飛行（a2_prefix、b2、2a_end）。
再利用3件のrun.jsonにあるoracle_biasは監査用の参考値で、この設定の推定入力・設定には使用していない。
manifest.jsonに設定、入力識別子、件数、SHA256を保存。
validation.jsonに全14飛行の時刻、有限性、初期化、LIO/真値不変の検査結果。
comparison.jsonに旧VIOと新VIOの初期整合後の誤差統計。
初期化期間を含む出力と推定器内部の診断データは../vio_audit/recomputed_diagnostics/へ分離。
このデータセットのtruth.csv/lio.csv/vio.csvは軌道追従期間だけを含む。
各replay.logとrun.jsonで入力配送を検証可能。画像と点群は含めない。
run1_middleの末尾2時刻は元入力の終端不足により出力なし。補作していない。

今回の再計算結果はCSV形式。共分散は保存していない。
../results/<label>/odometryのMCAPは旧設定の結果として保持。
その旧MCAPを再生しても今回の新VIOにはならない。画像/点群の元bagは不変。
新プロットは../error_plots/index.html、全14ページはall_flights.pdf。
旧プロットは../error_plots_before_vio_recompute/に保存。
上記の相対パスはローカル配置。Drive版の旧MCAPは親フォルダ直下の<label>/odometry。
診断データはローカルのvio_audit/recomputed_diagnostics/に保存。
''')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
