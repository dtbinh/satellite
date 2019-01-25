clear all
close all
clc

tmp = 'rw_blackbox';
mkdir(tmp);
cd(tmp);

fun = fullfile('/home/goh/Documents/20_MATLAB/bst_rwa_dm','rw_*.m');

pcode(fun);