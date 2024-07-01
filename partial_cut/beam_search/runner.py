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
    def run_all(self, seam_params=[], util_params=[]):
        for sp in seam_params:
            for up in util_params:
                print(f'\n--- TEST CASE util={up} seam={sp} ---\n')
                @patch('bsp.UTIL_WEIGHT', up)
                @patch('helpers.UTIL_WEIGHT', up)
                @patch('bsp.SEAM_WEIGHT', sp)
                @patch('helpers.SEAM_WEIGHT', sp)
                def patched_runner():
                    main()

                try:
                    patched_runner()
                except Exception as e:
                    write_file(e)



if __name__ == '__main__':
    runner = RunnerClass()
    runner.run_all(
        seam_params=[0.0001, 0.001, 0.01, 0.1],
        util_params=[0.01, 0.001, 0.0001],
    )
