#!usr/bin/env python

import sys
import csv
import os

def check_file(f, d, action='r'):
    '''
        @brief Checks if file exists in package in subdir of kalman_filter

        @param[in] filename (w/ or w/o .csv ending)
        @param[in] directory name beneath kalman_filter within which to search for f
        @param[in] r/w char for read/write, defaults to r
        @param[out] full path to properly named file if read and exist
        or if write and n_exist
    '''

    # Get path to data folder
    data_path = os.path.join( os.path.abspath(os.path.dirname(__file__)),
                              "../" + d + "/" )

    # Add file extension if necessary
    if not (f[-4:] == ".csv"):
        f = f + ".csv"

    # Create full filepath
    f = os.path.join( data_path, f )

    # Return file if it doesn't already exist
    if os.path.isfile( f ) and action == 'w': sys.exit(f + "\nFile exists!")
    elif not os.path.isfile( f ) and action == 'r': sys.exit(f + "\nFile does not exist!")
    
    return f
