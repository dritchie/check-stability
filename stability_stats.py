import argparse
from check_stability import check_stability
import os
import pandas as pd


def stability_stats(input_dir):
    data = []
    print(f'Checking stability of objects in {input_dir}')
    for entry in os.listdir(input_dir):
        base, ext = os.path.splitext(entry)
        if ext == '.obj':
            print(f'   Checking stability of {entry}...')
            is_stable = check_stability(f'{input_dir}/{entry}')
            data.append([entry, int(is_stable)])
    print('DONE')
    return pd.DataFrame(data, columns = ['Object Filename', 'Is Stable'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze stability of a bunch of different objects')
    # Path to input directory of OBJ files
    parser.add_argument('--input-dir', type=str, required=True)
    # Path to output CSV file where results will be written
    parser.add_argument('--output-file', type=str, default='stability_stats.csv')
    args = parser.parse_args()
    df = stability_stats(args.input_dir, args.output_file)
    df.to_csv(output_file, index=False)
    print(df.mean(), df.sem())