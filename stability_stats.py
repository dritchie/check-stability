import argparse
from check_stability import check_stability
import os
import pandas as pd


def stability_stats(input_dir, output_file):
    data = []
    for entry in os.listdir(input_dir):
        base, ext = os.path.splitext(entry)
        if ext == '.obj':
            is_stable = check_stability(f'{input_dir}/{entry}')
            data.append([entry, int(is_stable)])
    df = pd.DataFrame(data, columns = ['Object Filename', 'Is Stable'])
    print(df.mean())
    df.to_csv(output_file, index=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze stability of a bunch of different objects')
    # Path to input directory of OBJ files
    parser.add_argument('--input-dir', type=str, required=True)
    # Path to output CSV file where results will be written
    parser.add_argument('--output-file', type=str, default='stability_stats.csv')
    args = parser.parse_args()
    stability_stats(args.input_dir, args.output_file)