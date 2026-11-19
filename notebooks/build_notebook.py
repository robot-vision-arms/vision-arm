"""notebooks/model_comparison.py (셀 구분 '# %%') → model_comparison.ipynb

노트북은 diff 보기가 어려워서 소스는 .py 로 관리하고 이 스크립트로 만든다.
    .venv-ml/bin/python notebooks/build_notebook.py              # 만들기만
    .venv-ml/bin/python notebooks/build_notebook.py --execute    # 실행해서 결과·그래프까지 저장
"""
import argparse
from pathlib import Path

import nbformat

HERE = Path(__file__).resolve().parent


def cells(src: str):
    blocks, cur, kind = [], [], None
    for line in src.splitlines():
        if line.startswith('# %%'):
            if kind is not None:
                blocks.append((kind, cur))
            kind, cur = ('markdown' if '[markdown]' in line else 'code'), []
        else:
            cur.append(line)
    blocks.append((kind, cur))
    for kind, lines in blocks:
        while lines and not lines[-1].strip():
            lines.pop()
        if kind == 'markdown':
            yield nbformat.v4.new_markdown_cell('\n'.join(l[2:] if l.startswith('# ') else l.lstrip('#') for l in lines))
        else:
            yield nbformat.v4.new_code_cell('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--until', help="이 글자가 들어간 마크다운 셀 앞까지만 실행 (예: '## 5.'). 나머지 셀은 코드만 둔다")
    args = ap.parse_args()
    nb = nbformat.v4.new_notebook(cells=list(cells((HERE / 'model_comparison.py').read_text())))
    nb.metadata['kernelspec'] = dict(name='python3', display_name='Python 3 (.venv-ml)', language='python')
    out = HERE / 'model_comparison.ipynb'
    if args.execute:
        from nbconvert.preprocessors import ExecutePreprocessor
        cut = len(nb.cells)
        if args.until:
            cut = next(i for i, c in enumerate(nb.cells) if c.cell_type == 'markdown' and args.until in c.source)
        part = nbformat.v4.new_notebook(cells=nb.cells[:cut], metadata=nb.metadata)
        ExecutePreprocessor(timeout=None, kernel_name='python3').preprocess(part, {'metadata': {'path': str(HERE)}})
        nb.cells[:cut] = part.cells
    nbformat.write(nb, out)
    print('saved', out)


if __name__ == '__main__':
    main()
