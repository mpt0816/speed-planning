clc; clear all; close all;
%% 添加路径
current_folder = pwd;
addpath(genpath(current_folder));

InitCondition
Configs

tic;
planning_data_pid = PID();
toc; tic;
planning_data_qp = QP();
toc; tic;
planning_data_lattice = Lattice();
toc;

ThreeDResultComparison(planning_data_pid, planning_data_qp, planning_data_lattice);