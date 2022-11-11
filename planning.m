clc; clear all; close all;
%% 添加路径
current_folder = pwd;
addpath(genpath(current_folder));

InitCondition
Configs

planning_data_pid = PID();
planning_data_qp = QP();

ResultComparison(planning_data_pid, planning_data_qp);