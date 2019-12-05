% MAKESFUNCTION Compile the LTV-MPC S-Function.
clear; clc;

%% Add header files
% Add path to Eigen header files
eigen_path = fullfile('..','lib','eigen','Eigen');
ipath_ekigen = ['-I' eigen_path];

% Add path to QpOses header files
qpoases_path = fullfile('..','lib','qpoases','include');
ipath_qpoases0 = ['-I' qpoases_path];
ipath_qpoases1 = ['-I' fullfile(qpoases_path,'qpOASES')];
ipath_qpoases2 = ['-I' fullfile(qpoases_path,'qpOASES','extras')];


%% Add library files
% % qpOASES
% lpath_libqpOASES_dir  = ['-L' fullfile('..','build','lib')];
% lpath_libqpOASES_name = ['-l:' 'qpOASES'];
% 
% % LTVMPC
% lpath_libltvmpc_dir  = ['-L' fullfile('..','build','lib')];
% lpath_libltvmpc_name = ['-l:' 'libltvmpc'];

% qpOASES
lpath_libqpOASES_name = [fullfile('..','build','lib') '/' 'libqpOASES.a'];

% LTVMPC
lpath_libltvmpc_name = [fullfile('..','build','lib') '/' 'liblibltvmpc.a'];

%% Build the MEX file
% Option
verbose = true;
debug = true;

% Build command
mexarg = {'CXXFLAGS="$CXXFLAGS -std=c++14"', ...
    ...lpath_libltvmpc_dir, lpath_libltvmpc_name, ...
    ...lpath_libqpOASES_dir, lpath_libqpOASES_name, ...
    lpath_libltvmpc_name, ...
    lpath_libqpOASES_name, ...
    ipath_eigen, ipath_qpoases0, ipath_qpoases1, ipath_qpoases2, ...
    'ltvmpc_sfunc.cpp'...
    };
if verbose
    mexarg = [mexarg{:}, {'-v'}];
end
if debug
    mexarg = [mexarg{:}, {'-g'}];
end

% Run mex
mex(mexarg{:})


% disp('Copy file')
% copyfile ltvmpc_sfunc.mexa64 ../../matlab/ltvmpc_sfunc.mexa64


