import argparse
from check_rooted import check_rooted
import os
import pandas as pd


def rooted_stats(input_dir, output_file=None):
    data = []
    print(f'Checking rooted-ness of objects in {input_dir}')
    for entry in os.listdir(input_dir):
        base, ext = os.path.splitext(entry)
        if ext == '.obj':
            print(f'   Checking rootedness of {entry}...')
            is_stable = check_rooted(f'{input_dir}/{entry}')
            data.append([entry, int(is_stable)])
    print('DONE')
    return pd.DataFrame(data, columns = ['Object Filename', 'Is Rooted'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze rooted-ness of a bunch of different objects')
    # Path to input directory of OBJ files
    parser.add_argument('--input-dir', type=str, required=True)
    # Path to output CSV file where results will be written
    parser.add_argument('--output-file', type=str, default='rooted_stats.csv')
    args = parser.parse_args()
    df = rooted_stats(args.input_dir, args.output_file)
    df.to_csv(output_file, index=False)
    print(df.mean(), df.sem())