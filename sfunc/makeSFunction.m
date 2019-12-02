clear; clc;

%% Add header files
% Add path to Eigen header files
eigen_path = fullfile('..','lib','eigen','Eigen');
ipath_eigen = ['-I' eigen_path];

% Add path to QpOses header files
qpoases_path = fullfile('..','lib','qpoases','include');
ipath_qpoases0 = ['-I' qpoases_path];
ipath_qpoases1 = ['-I' fullfile(qpoases_path,'qpOASES')];
ipath_qpoases2 = ['-I' fullfile(qpoases_path,'qpOASES','extras')];


%% Add library files
% qpOASES
lpath_libqpOASES_dir  = ['-L' fullfile('..','build','lib')];
lpath_libqpOASES_name = ['-l' 'qpOASES'];

% LTVMPC
lpath_libltvmpc_dir  = ['-L' fullfile('..','build','lib')];
lpath_libltvmpc_name = ['-l' 'libltvmpc'];

%% Build the MEX file
% Option
verbose = false;

% Build command
mexarg = {'CXXFLAGS="$CXXFLAGS -std=c++14"', ...
    lpath_libltvmpc_dir, lpath_libltvmpc_name, ...
    lpath_libqpOASES_dir, lpath_libqpOASES_name, ...
    ipath_eigen, ipath_qpoases0, ipath_qpoases1, ipath_qpoases2, ...
    'ltvmpc_sfunc.cpp'...
    };
if verbose
    mexarg = [mexarg{:}, {'-v'}];
end

% Run mex
mex(mexarg{:})




