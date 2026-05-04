%% SEAGLIDER D.A.P. system 

% The goal of this code is to diagnose and isolate a fault on a seaglider

%% Inputs
% Log and Eng files from desired mission

%% Outputs
% fault diagnose of wing or rudder and percentage loss

%% Authors
% Jordan Cummings, Henry Hong, Holland Kanter, Letizia Laura, Edward Park, 
% Oleksiy Polyakov, Geenadie Rathnayake, Joshua Rolfe, Mak Sukimoto, 
% Dante Weerasooriya, Kyle Wittcoff

%% Updated on
% 05/03/2026

%% Files required
% log_files, eng_files, DAP_Main.m, dynamicsV3.m, FaultIsolator.m,
% missioncompare.m, nc_file_reader_function.m, nc_file_reader_script.m,
% parameters.m

clear
clc

%% Log file unpacking
% This section unpacks the log file into a matrix

Log_Matrix = Log_File_Unpacker('p1950004.log'); % USER INPUT - insert name of log file into the ('');   Example - ('p1950001.log') 


%% ENG file unpacking
% This section unpacks the eng file into a matrix

Eng_Matrix = Eng_File_Unpacker('p1950004.eng'); % USER INPUT - insert name of eng file into the ('');   Example - ('p1950001.eng') 


%% Dynamics


Nom_Sim_Matrix = Create_Dynamics_Matrix(Log_Matrix, Eng_Matrix, coefficientsV3, parametersV3);


%% Diagnoser

% Compare simulated nominal mission to actual ENG mission
Diagnose = missioncompare(Nom_Sim_Matrix, Eng_Matrix);

%% Display Diagnose Results
disp(Diagnose);




















