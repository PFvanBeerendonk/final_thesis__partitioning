from unittest import skip
from unittest.mock import patch
from unittest import TestCase

from datetime import datetime
from config import OUTPUT_FOLDER
import os

from helpers import write_statfile_to_export
from main import main

def write_file(e):
    now = datetime.now()
    current_location = os.path.dirname(__file__)
    path_to_files = f'{current_location}/{OUTPUT_FOLDER}/{now.strftime("%m-%d-%Y--%H-%M")}'
    os.mkdir(path_to_files)
    write_statfile_to_export(path_to_files, 'error', str(e))


class RunnerClass(TestCase):
    def run_all(self, params=[]):
        for pair in params:
            input_file_path, beam, space = pair
            print(f'\n--- MODEL {input_file_path} --- \n\n\n' )
            print(f'\n--- TEST CASE beam_width={beam} plane space={space} mm ---\n')

            # define patched runner as main with @patch-es on it
            @patch('main.INPUT_FILE_PATH', input_file_path)
            @patch('beam_search.BEAM_WIDTH', beam)
            @patch('helpers.PLANE_SPACER', space)
            def patched_runner():
                main()

            try:
                patched_runner()
            except Exception as e:
                write_file(e)


if __name__ == '__main__':
    runner = RunnerClass()

    runner.run_all(
        params=[
            ('../../models/tue_logo.stl',  1, 10),
            ('../../models/propeller.stl', 1, 10),
            ('../../models/spikes.stl',    1, 10),
        ],
    )

    # beam in: [1,2,3,4,5]
    # space in: [10, 5, 4, 3, 2, 1]
