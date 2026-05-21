% Dynamics Test Script #3: Nominal Dive Phase
% This script applies the dynamics function to a window of the ascent phase
% where actuator states do not change. 
% It compares the result to the recorded states at the respective times


%% Boring Initialization Stuff

% Clears 
clear;close;clc 

% Load data file from Edmonds dive 5
% Supports either the normal filename or the uploaded ChatGPT filename.
if exist('Dive5_data.mat','file')
    load('Dive5_data.mat');
elseif exist('Dive5_data(2).mat','file')
    load('Dive5_data(2).mat');
else
    error('Could not find Dive5_data.mat or Dive5_data(2).mat in the current folder.');
end
mdmat = Mission_Data_Matrix(2:end,:); % renaming to be shorter and trimming header
mdmat = cell2mat(mdmat); % converting to double

% Starting and ending data point of dive
start = 59;
fin = 65;

% Data points we're looking at for dive (t = 872s to t= 932s)
% Time steps between data points
for i = 1:7
    dts(i) = mdmat(start+i,2) - mdmat(start-1+i,2);
end

%% Parameters

% Reading parameter/control histories from the mission-data window.
% These histories are useful for checking that the selected window really has
% nearly constant controls, but dynamicsV5 expects one scalar control vector
% and scalar environmental parameters for a single ODE solve. Passing the full
% histories makes q, B, Feq, and Teq become row/vector-valued and causes a
% downstream H*Omegadot dimension error.
test_idx_window = start:fin;
x_bat_hist  = mdmat(test_idx_window,7) / 100;  % battery position (cm to m)
phi_bat_hist = mdmat(test_idx_window,8);       % battery roll, deg
vbdCC_hist   = mdmat(test_idx_window,9);       % VBD oil count
rho_hist     = mdmat(test_idx_window,14);      % water density, kg/m^3

% Use the first point in the fixed-control window as the constant ODE input.
control_idx = start;
x_bat_const   = mdmat(control_idx,7) / 100;
phi_bat_const = mdmat(control_idx,8);
vbdCC_const   = mdmat(control_idx,9);
rho_const     = mdmat(control_idx,14);

control_readout = table(test_idx_window(:), x_bat_hist(:), phi_bat_hist(:), ...
                        vbdCC_hist(:), rho_hist(:), ...
                        'VariableNames', {'mdmat_row','x_bat_m', ...
                        'phi_bat_deg','vbdCC','rho_kg_m3'});
disp('Control/environment values across selected mission-data window:');
disp(control_readout);
disp('Using first row of this window as constant U and rho for ode45.');

% Target / mission parameters
params.heading_desired = mdmat(start,4); % deg, desired heading for dive

% Aerodynamic / hydrodynamic geometry
params.S    = 0.486;              % m^2, wing/reference surface area [TBD]
params.cbar = 0.173;              % m, mean aerodynamic chord [TBD]
params.b    = 1.00;              % m, wing span [TBD]

% Battery properties
params.mbat  = 11.636;             % kg, battery pack mass [TBD]

% Whole glider properties
params.Vstatic = 0.0562;        % m^3, displaced volume without VBD [TBD]
params.Ms      = 58.852 - params.mbat;           % kg, stationary mass [TBD]

% Added mass matrix, kg
params.Mf = [3.310 0 0
            0 72.005 0
            0 0 72.005];          % estimated added mass

% Added mass inertia matrix, kg*m^2
params.Jf = [0 0 0
             0 10.062 0
             0 0 10.062];          % estimated added inertia

% Stationary mass inertia matrix, kg*m^2
params.Js = [0.1861 -0.0007 0.1095
            -0.0007 8.1735 -0.0000
             0.1095 -0.0000 8.2073]; % from trim sheet, may need CAD

% Ocean / environmental properties
params.ambtemp  = mdmat(1,11); % deg C, ambient surface temperature [TBD]
params.temp     = mdmat(60,11); % deg C, local water temperature [TBD]
params.pressure = 0; % pressure/depth input used in dynamicsV3 [TBD]
params.rho      = rho_const; % kg/m^3, scalar seawater density for this ODE run

%% Coefs (copied from coefs function)
% Alpha and beta lookup grids
coefs.alphas = [-15 -14 -13 -12 -11 -10 -7 -5 -4 -2 0 2 4 5 7 10 11 12 13 14 15];   % deg
coefs.betas  = [0 2 4 5 7 10];   % deg

% CLs coefficient matrix
coefs.CLs = [ ...
    -0.01920351 -0.014935595 -0.0152662005 -0.0124014583 -0.0116306196  0.000184897987;   % alpha = -15
    -0.01659005 -0.015238798 -0.0126780652 -0.0146192159 -0.0115160036 -0.00860370961;   % alpha = -14
    -0.01649774 -0.014384304 -0.014005882 -0.0124077573 -0.00659507954  0.000700399178;   % alpha = -13
    -0.01480433 -0.014057322 -0.014204769 -0.014876673 -0.00684741454 -0.0144487775;   % alpha = -12
    -0.0143142 -0.01368786 -0.0116281398 -0.0128462165 -0.0100951845  0.000565079212;   % alpha = -11
    -0.01301843 -0.01267137 -0.0122817241 -0.0122652597 -0.00359088807 -0.00515111379;   % alpha = -10
    -0.00954613 -0.010566865 -0.00997818484 -0.0106271884 -0.00941670611 -0.00469525892;   % alpha = -7
    -0.0067083 -0.008200668 -0.00883252965 -0.00860805419 -0.00845499297 -0.00716695325;   % alpha = -5
    -0.00713402 -0.00690939 -0.00690997399 -0.00807740764 -0.0072715799 -0.00841561545;   % alpha = -4
    -0.00581969 -0.00621306 -0.00497191504 -0.00557086366 -0.00589703547 -0.00450177821;   % alpha = -2
    -0.002525 -0.0015018 -0.00336866 -0.00321919 -0.0042770129 -0.00289752971;   % alpha = 0
    -0.001742 -0.00221273 -0.0015616 -0.00138997 -0.00119401196 -0.00126171687;   % alpha = 2
     0.00038638  0.0003322776  0.000442652 -0.0002825466  0.000291276558  0.00196189677;   % alpha = 4
     0.002002  0.0003325176  0.0017718  0.001144886  0.00127108557  0.00035294172;   % alpha = 5
     0.0040117  0.001964471  0.00292317  0.0036851 -0.00324899765 -0.000269007524;   % alpha = 7
     0.0063721  0.00576582  0.0049384  0.00577879  0.00537035769  0.00157896729;   % alpha = 10
     0.0066438  0.00668538  0.0079804  0.00512471  0.000182165535 -0.000208063645;   % alpha = 11
     0.0086314  0.008011333  0.00771367  0.00676016  0.00187924158  0.0040229514;   % alpha = 12
     0.00934172  0.007811365  0.00451868  0.00614801  0.00919545061 -0.00568872356;   % alpha = 13
     0.01059841  0.007800297  0.00816249  0.00740263  0.00402228612  0.0059117007;   % alpha = 14
     0.0116445  0.009055499  0.00556038  0.00488935  0.00161985734 -0.00737049519;   % alpha = 15
    ];

% CDs coefficient matrix
coefs.CDs = [ ...
     0.01418396  0.0137852376  0.0135755906  0.0133014121  0.0133800384  0.0118777193;   % alpha = -15
     0.0138945  0.013849037  0.0136247814  0.013395869  0.0130659368  0.0125894747;   % alpha = -14
     0.01373939  0.013812335  0.0135368335  0.0133701146  0.0118134971  0.0145067738;   % alpha = -13
     0.01354999  0.013688364  0.013700653  0.0131644615  0.0136679564  0.0130294627;   % alpha = -12
     0.0137913  0.013666179  0.013513547  0.0133387143  0.0126992044  0.010358903;   % alpha = -11
     0.0134782  0.01378803  0.0135012761  0.013410762  0.0137107743  0.0110106588;   % alpha = -10
     0.01342157  0.01350889  0.0134319788  0.013797302  0.0130331294  0.0119618825;   % alpha = -7
     0.01347731  0.01343135  0.013650611  0.013550153  0.0139583454  0.0135294639;   % alpha = -5
     0.013546  0.013359984  0.0133476909  0.0139503751  0.0135414604  0.0138724556;   % alpha = -4
     0.0136687  0.013610094  0.0135526499  0.013766053  0.0135927821  0.013828204;   % alpha = -2
     0.01324  0.017322  0.0135657  0.0136396  0.0139876894  0.0139279465;   % alpha = 0
     0.01342  0.01333127  0.0135052  0.013583  0.0137925905  0.0141165794;   % alpha = 2
     0.01329  0.01362347  0.013629  0.0139551  0.0140236387  0.0138714294;   % alpha = 4
     0.013886  0.0137188  0.013381  0.0135918  0.0137303624  0.0136887483;   % alpha = 5
     0.013529  0.013899467  0.0136006  0.0136994  0.0132930929  0.0126160271;   % alpha = 7
     0.013494  0.01403611  0.0140725  0.0138155  0.0131535306  0.0119231552;   % alpha = 10
     0.013997  0.01415414  0.013524  0.01309287  0.0132138084  0.0141773792;   % alpha = 11
     0.013841  0.01418419  0.01384748  0.0137132  0.0133899835  0.0130943378;   % alpha = 12
     0.0139544  0.01438429  0.0139965  0.0132942  0.0124974524  0.011855159;   % alpha = 13
     0.0141331  0.014194823  0.01389776  0.0139719  0.0132944786  0.0121357687;   % alpha = 14
     0.0146038  0.01440539  0.0140905  0.01420757  0.0135761591  0.0123712486;   % alpha = 15
    ];

% CYs coefficient matrix
coefs.CYs = [ ...
     0.001123527  0.000581000411  0.00252235844  0.00162053751  0.0030755244 -0.000659857694;   % alpha = -15
     0.0005135129  0.001302019  0.00242541131  0.00244304724  0.00291065868  0.00233927925;   % alpha = -14
     0.0001900065  0.000738949  0.00189621479  0.00299334485  0.000138685335  0.00327683304;   % alpha = -13
     0.000899737  0.0019584956  0.00157892686  0.00316415172  0.00127258678  0.00226311007;   % alpha = -12
     0.000767982  6.73847299e-4  0.000917387867  0.000523138305  0.00159895302 -0.00168516659;   % alpha = -11
    -0.000449523 -0.000100908  0.00153964197  0.00135614701 -0.000547371838 -0.000934192823;   % alpha = -10
    -0.00183361  0.0007827537  0.00276414463  0.0003651076  0.00273171097  0.0024312304;   % alpha = -7
    -0.000404666  0.001001825  0.000930044905  0.00188082655  0.0019579084  0.00071863783;   % alpha = -5
     0.00065362  0.000659765  0.00148597285  0.00145354547  0.0011155753  0.00218205706;   % alpha = -4
     0.000464948 -7.926229e-05  0.00147631827  0.000902923682  0.000654842194  0.00125204922;   % alpha = -2
     0 -0.1670882  0.0011452592 -6.302488e-05  0.00139444936  0.00187748939;   % alpha = 0
     0  0.0333069  0.001351  0.00086055427  0.00139600478  0.00265228326;   % alpha = 2
     0  0.0474675  0.001547  0.0019734  0.00141438784  0.00203684859;   % alpha = 4
     0  0.0155958  0.0011534  0.000120027  0.00173003304  0.00240525909;   % alpha = 5
     0  0.002170668  0.00193116  0.000345148 -0.00119477653  0.00211931805;   % alpha = 7
     0  0.00091145989  0.0019877  0.00133981  0.00217782805  0.00192528733;   % alpha = 10
     0  0.0008815978  0.0011625  0.00114875 -0.000341128544  0.00235470367;   % alpha = 11
     0  0.00158365  0.00215775  0.00224129  0.000810804882  0.00264619983;   % alpha = 12
     0  0.000799806  0.00074978  0.001404358  0.00050748729 -0.0027124778;   % alpha = 13
     0  0.002225194  0.00268859  0.00278176  0.00196721137  0.000197840566;   % alpha = 14
     0  0.00091380497  0.0019448  0.0046187  0.00276436244 -0.00534707909;   % alpha = 15
    ];

% Croll coefficient matrix
coefs.Croll = [ ...
     4.92586e-05 -0.000271713868 -0.000630071871 -0.000604535861 -0.000947568109 -0.000680829401;   % alpha = -15
     3.434224e-05 -0.00022513 -0.000411299303 -0.000619090079 -0.000628403922 -0.000816848834;   % alpha = -14
     3.9173e-05 -0.000239043 -0.000439178406 -0.000477038253 -0.000305640105 -0.000610020333;   % alpha = -13
     5.69057e-05 -0.000207096399 -0.000396243219 -0.000578497827 -0.000278969017 -0.00122206267;   % alpha = -12
     8.00571e-05 -0.0001862988 -0.000282573583 -0.000526478886 -0.00029131109 -0.000173556973;   % alpha = -11
     2.941e-05 -0.0001848447 -0.000337120047 -0.000424761048 -0.000191430151 -0.000337156701;   % alpha = -10
     2.17669e-05 -0.000116648 -0.000184961493 -0.000306499174 -0.000308145908 -0.000224458285;   % alpha = -7
     3.38171e-05 -1.6180688e-05 -0.000115707829 -0.000178666317 -0.000240160874 -0.000379815326;   % alpha = -5
     4.334566e-05 -4.6517e-05 -0.000133650622 -0.000185124854 -0.000270616858 -0.000299552381;   % alpha = -4
     6.0924082e-05 -5.209127e-05 -1.7441049e-05 -6.66484141e-05 -0.000156074319 -0.000225497646;   % alpha = -2
     3.5285e-05  0.0001006539 -5.683936e-05 -2.521526e-06 -4.1196e-05 -1.86664938e-05;   % alpha = 0
     1.6246e-06  3.126397e-05  1.40416e-05  6.127408e-05  6.99281998e-05  0.000178625261;   % alpha = 2
    -6.2948e-05  0.000114166328  0.0001235996  0.0001582899  0.000168247639  0.000327253153;   % alpha = 4
    -3.5798e-05  8.402247e-05  0.00013151  0.0001344549  0.000279053368  0.000289088511;   % alpha = 5
    -1.68237e-05  0.0001515112  0.000248987  0.00018423  1.60358717e-05  0.000321804172;   % alpha = 7
    -3.73159e-06  0.00014680162  0.0002932988  0.000395456  0.000508307785  0.000551790622;   % alpha = 10
    -1.051316e-05  0.000201672  0.000331072  0.000329397  0.000273634918  0.000483997778;   % alpha = 11
     2.27584e-06  0.00017255371  0.000417593  0.000471735  0.000386262445  0.000804894096;   % alpha = 12
     8.10517e-06  0.000196656  0.0003245004  0.00042149  0.000646709696  0.000528581967;   % alpha = 13
     1.92498e-05  0.0002458418  0.000528436  0.000643546  0.000736380179  0.000983352614;   % alpha = 14
     1.48191e-05  0.0002745408  0.000521024  0.0007771773  0.000787080361  0.000319746802;   % alpha = 15
    ];

% Cpitch coefficient matrix
coefs.Cpitch = [ ...
    -0.004761566  0.002154635  0.00204518751  0.00568150305  0.00889205986  0.012206402;   % alpha = -15
    -0.0021332599  0.0007129126  0.00145334197  0.000279873281  0.00898862439  0.00271767233;   % alpha = -14
    -0.00175491  0.00308328  0.00122178491 -0.00544189111 -0.00418184707  0.0277306231;   % alpha = -13
    -0.00427444 -0.003268507  0.00214514965 -0.00285106189  0.00876463256  0.0099005559;   % alpha = -12
    -0.0034929  0.00177148  0.00364222955  0.0086071349 -0.00299563272 -0.00491719343;   % alpha = -11
     5.948e-05  0.00427359  0.0022710152  0.00267857501  0.0124718908 -0.00553653525;   % alpha = -10
     0.00659413 -0.000261978 -0.006407784  0.00523402591 -0.00315861273 -0.00811463576;   % alpha = -7
     0.00144704 -0.0030579  0.00130302183 -0.000874799349  0.00107783332  0.00817401634;   % alpha = -5
    -0.00422383 -0.0009761475 -0.00135227203 -1.51227652e-05  0.00518439808  0.003786416;   % alpha = -4
    -0.00296146  0.00192526 -0.00190427322  0.00247732903  0.00514007809  0.00860680006;   % alpha = -2
    -0.004731  0.03089052  0.001026269  0.00582199  0.0032690434  0.00524963315;   % alpha = 0
    -0.00526 -0.00268604 -0.000732468  0.0030696  0.00263316506  0.0020315128;   % alpha = 2
    -0.002749 -0.004404489 -0.000873388 -0.00175248  0.00360087773  0.0059839256;   % alpha = 4
    -0.001419  9.551448e-05  0.00050964  0.00609597  0.00180809196  0.00172808619;   % alpha = 5
     0.0019977 -0.00706162 -0.00247225  0.00701716  0.00887504191 -0.00095690203;   % alpha = 7
     0.000480231 -0.00044403945 -0.00172449  0.0040604  0.000894811512 -0.00222877022;   % alpha = 10
    -0.00184227  0.00038814995  0.0030511944 -0.000452918  0.00701272296  0.0126862985;   % alpha = 11
    -0.000718793 -0.0015541212  0.0004329963 -0.000384725  0.0105060554  0.00443704736;   % alpha = 12
    -0.000589943  0.002488319  0.00222899  0.004517288  0.00103158928  0.00991377472;   % alpha = 13
    -0.0051761 -0.00564818 -0.0021396  0.00413051  0.0021018666  0.0134976499;   % alpha = 14
    -0.0015537  0.003129229  0.004062831 -0.00116526  0.00622221375  0.0156890545;   % alpha = 15
    ];

% Cyaw coefficient matrix
coefs.Cyaw = [ ...
    -0.0034363 -0.00269011226 -0.00292645546 -0.00239105613 -0.00243845151 -0.00120326063;   % alpha = -15
    -0.002831902 -0.00263711 -0.00237214464 -0.00268251849 -0.00279433548 -0.00214049547;   % alpha = -14
    -0.00320123 -0.002475101 -0.00279085085 -0.00245893738 -0.00164550475 -0.00208397821;   % alpha = -13
    -0.002771603 -0.002469061 -0.00264908329 -0.00267039897 -0.000990475906 -0.00280846823;   % alpha = -12
    -0.00266002 -0.002509 -0.00225082943 -0.00219773806 -0.0020568489 -0.000786974829;   % alpha = -11
    -0.00254215 -0.00223867 -0.00228576484 -0.00234942684 -0.000301823089 -0.0015888223;   % alpha = -10
    -0.00182312 -0.002085574 -0.00197235972 -0.00202763454 -0.00191569512 -0.00157277236;   % alpha = -7
    -0.0010661 -0.001574884 -0.00179653997 -0.00171115567 -0.00150700077 -0.00129523674;   % alpha = -5
    -0.001364529 -0.00134351 -0.00134987215 -0.00141884754 -0.00141404801 -0.00169096722;   % alpha = -4
    -0.001187 -0.00128319 -0.000872193945 -0.00105371607 -0.00138386302 -0.000767926431;   % alpha = -2
    -0.00051284 -0.0004547414 -0.0006525193 -0.00072303 -0.000790645 -0.000619091561;   % alpha = 0
    -0.00032809 -0.00061019907 -0.0004090268 -0.00033661 -0.000304765421 -0.000389162711;   % alpha = 2
    -2.57495e-05 -7.38658e-05  1.300623e-05 -6.547804e-05 -1.14717479e-05  0.000119747831;   % alpha = 4
    -0.00011885 -6.508974e-05  0.00016836 -1.635656e-05  6.44315737e-05  7.58749538e-05;   % alpha = 5
     0.000784823  0.00021731798  0.0003404804  0.000603266 -0.000791978114  0.000335702468;   % alpha = 7
     0.001007  0.00100652  0.0009551788  0.00101828  0.000949215483  0.000769633033;   % alpha = 10
     0.0010859  0.00107877  0.0014814  0.0010509  0.000195641779 -0.000784539275;   % alpha = 11
     0.0013794  0.00138644  0.001365814  0.00111062  0.000155280598  0.000980510141;   % alpha = 12
     0.0014688  0.00128718  0.000781306  0.00176133  0.00187100711 -0.000542151089;   % alpha = 13
     0.00160681  0.001449943  0.00159198  0.00103235  0.000972170576  0.00153848009;   % alpha = 14
     0.00170503  0.00142532  0.0009168034  0.000694395  0.000233014936 -0.00113943531;   % alpha = 15
    ];

coefs.Croll = ones(21,6)*1e-9;
coefs.Citch = ones(21,6)*1e-9;
coefs.Cyaw = ones(21,6)*1e-9;
coefs.CLs = ones(21,6)*1e-9;
coefs.CDs = ones(21,6)*1e-9;
coefs.CYs = ones(21,6)*1e-9;
%% Building initial state vector

% dt0 = mdmat(59,2) - mdmat(58,2); 
dt0 = dts(1); %dt for angular rates

phidot0 = (mdmat(start,6) - mdmat(start-1,6)) / dt0; % roll rate
thetadot0 = (mdmat(start,5) - mdmat(start-1,5)) / dt0; % pitch rate
headingdot0 = (mdmat(start,4) - mdmat(start-1,4)) / dt0; % heading rate

phi0 = mdmat(start,6); % initial roll angle
theta0 = mdmat(start,5); % initial pitch angle
heading0 = mdmat(start,4); % initial heading

u0 = mdmat(start,15) / 100; % initial GSM horizontal speed (cm/s to m/s)
v0 = 0; % initial lateral speed (ASSUMED, not KNOWN)
w0 = mdmat(start,10) / 100 * -1; % intial GSM vertical speed (cm/s to m/s)

x0 = 0; % set to 0 because it is not measured and doesn't affect test
y0 = 0; % set to 0 because it is not measured and doesn't affect test
z0 = mdmat(start,3) / 100; % initial depth (cm/s to m/s and neg to pos)

X0 = [phidot0; thetadot0; headingdot0; phi0; theta0; heading0; u0; v0;...
      w0; x0; y0; z0;]; % X0 vector
 


%% Building Initial Control vector 

% Picking one point for each because we chose a window with no control
% changes




% x_bat = mdmat(start,7) / 100; % battery lateral pos (cm to m)
% phi_bat = mdmat(start,8); % battery roll
% vbdCC = mdmat(start,9);

% Constant control vector for this fixed-control test window.
% dynamicsV5 indexes U(1), U(2), U(3), so keep this as 3x1.
U = [x_bat_const; phi_bat_const; vbdCC_const];


%% Running ODE45

dt = mdmat(fin,2) - mdmat(start,2);

[t, X] = ode45(@(t,X) dynamicsV6(X, U, coefs, params), [0 dt], X0);



% Trimming angles to 360deg
X(:,6) = mod(X(:,6),360);

%% Alpha / Beta Readout from ODE45 Returned Steps
% Recompute alpha and beta from each accepted ODE45 solution point using
% the same body-frame velocity transform used inside dynamicsV5.
% Note: t and X from ode45 are accepted solver output steps, not every
% rejected RHS evaluation attempted internally by the adaptive solver.

nOdeSteps = numel(t);
Step = (1:nOdeSteps).';
Time_s = t(:);

alpha_deg = zeros(nOdeSteps,1);
beta_deg  = zeros(nOdeSteps,1);
alpha_clamp_deg = zeros(nOdeSteps,1);
beta_clamp_deg  = zeros(nOdeSteps,1);
V_mag = zeros(nOdeSteps,1);
ub_readout = zeros(nOdeSteps,1);
vb_readout = zeros(nOdeSteps,1);
wb_readout = zeros(nOdeSteps,1);

heading_desired_rad = deg2rad(params.heading_desired);

for k = 1:nOdeSteps
    % Euler angles from state vector, converted to radians
    phi_k     = deg2rad(X(k,4));
    theta_k   = deg2rad(X(k,5));
    heading_k = deg2rad(X(k,6));

    % Glide-path-frame velocity from state vector
    V_g_k = [X(k,7); X(k,8); X(k,9)];

    % Same yaw convention as dynamicsV5
    psi_k = heading_k - heading_desired_rad;

    % Same DCM convention as dynamicsV5: glide path frame to body frame
    DCM_gb_k = angle2dcm(psi_k, theta_k, phi_k);
    V_b_k = DCM_gb_k * V_g_k;

    ub_readout(k) = V_b_k(1);
    vb_readout(k) = V_b_k(2);
    wb_readout(k) = V_b_k(3);

    V_mag(k) = sqrt(sum(V_g_k.^2));

    if V_mag(k) > 1e-6
        alpha_deg(k) = atan2(wb_readout(k), ub_readout(k)) * 180/pi;
        beta_deg(k)  = asin(vb_readout(k) / V_mag(k)) * 180/pi;
    else
        alpha_deg(k) = 0;
        beta_deg(k)  = 0;
    end

    % These are the values actually sent into the coefficient lookup table.
    alpha_clamp_deg(k) = min(max(alpha_deg(k), min(coefs.alphas)), max(coefs.alphas));
    beta_clamp_deg(k)  = min(max(beta_deg(k),  min(coefs.betas)),  max(coefs.betas));
end

alpha_beta_readout = table(Step, Time_s, ...
                           X(:,7), X(:,8), X(:,9), ...
                           ub_readout, vb_readout, wb_readout, V_mag, ...
                           alpha_deg, beta_deg, ...
                           alpha_clamp_deg, beta_clamp_deg, ...
                           'VariableNames', {'ODE45_step','t_s', ...
                           'u_g_mps','v_g_mps','w_g_mps', ...
                           'u_b_mps','v_b_mps','w_b_mps','V_mps', ...
                           'alpha_deg','beta_deg', ...
                           'alpha_clamped_deg','beta_clamped_deg'});

disp(' ');
disp('Alpha / Beta Readout for each returned ODE45 step:');
disp(alpha_beta_readout);

% Save alpha/beta readout for easier inspection if the command-window output
% is long.
writetable(alpha_beta_readout, 'alpha_beta_readout.csv');

%% Align engineering data to ODE time base

% Test data rows in mdmat. Since mdmat = Mission_Data_Matrix(2:end,:),
% Mission_Data_Matrix row number = mdmat row number + 1.
test_idx = start:fin;

eng_elaps_t  = mdmat(test_idx,2);              % absolute elapsed time, s
test_t       = eng_elaps_t - mdmat(start,2);   % relative time used by ode45, s

eng_depth    = mdmat(test_idx,3) / 100;        % cm to m, positive depth
eng_w        = mdmat(test_idx,10) / 100 * -1;  % cm/s to m/s, same sign convention as w0

eng_head     = mod(mdmat(test_idx,4),360);     % deg
eng_pitchang = mdmat(test_idx,5);              % deg
eng_rollang  = mdmat(test_idx,6);              % deg

% Numerical rate of change of engineering Euler angles
eng_heading_unwrapped = unwrap(eng_head*pi/180) * 180/pi;
eng_thetadot   = gradient(eng_pitchang, test_t);            % deg/s
eng_phidot     = gradient(eng_rollang,  test_t);            % deg/s
eng_headingdot = gradient(eng_heading_unwrapped, test_t);   % deg/s

% Print alignment table
test_point_map = table((1:numel(test_idx))', test_idx', (test_idx'+1), ...
                       eng_elaps_t, test_t, ...
                       'VariableNames', {'TestPoint','mdmat_row', ...
                       'Mission_Data_Matrix_row','eng_elaps_t_s', ...
                       'relative_t_s'});
disp('Test point alignment: ODE t = eng_elaps_t - eng_elaps_t(start)');
disp(test_point_map);

%% Error Evaluation

% Allowed error:
% 6.6% for depth, pitch, roll, heading
% 3.3% for vertical velocity, pitch rate, roll rate, heading rate

% Interpolate modeled/simulated data onto mission-data time points
sim_depth = interp1(t, X(:,12), test_t, 'linear', 'extrap');
sim_w     = interp1(t, X(:,9),  test_t, 'linear', 'extrap');

sim_pitch = interp1(t, X(:,5), test_t, 'linear', 'extrap');
sim_roll  = interp1(t, X(:,4), test_t, 'linear', 'extrap');

% Unwrap heading/yaw to avoid false 0/360 deg jumps
sim_heading_unwrapped_all = unwrap(X(:,6)*pi/180) * 180/pi;
sim_heading = interp1(t, sim_heading_unwrapped_all, test_t, 'linear', 'extrap');

% Shift simulated heading near mission heading
sim_heading = sim_heading + 360*round((eng_heading_unwrapped(1) - sim_heading(1))/360);

sim_pitch_rate   = interp1(t, X(:,2), test_t, 'linear', 'extrap');
sim_roll_rate    = interp1(t, X(:,1), test_t, 'linear', 'extrap');
sim_heading_rate = interp1(t, X(:,3), test_t, 'linear', 'extrap');

% Variables being checked
varNames = ["depth";
            "vertical_velocity";
            "pitch";
            "roll";
            "heading";
            "pitch_rate";
            "roll_rate";
            "heading_rate"];

varUnits = ["m";
            "m/s";
            "deg";
            "deg";
            "deg";
            "deg/s";
            "deg/s";
            "deg/s"];

% Mission data
missionData = {eng_depth(:);
               eng_w(:);
               eng_pitchang(:);
               eng_rollang(:);
               eng_heading_unwrapped(:);
               eng_thetadot(:);
               eng_phidot(:);
               eng_headingdot(:)};

% Simulated data
simData = {sim_depth(:);
           sim_w(:);
           sim_pitch(:);
           sim_roll(:);
           sim_heading(:);
           sim_pitch_rate(:);
           sim_roll_rate(:);
           sim_heading_rate(:)};

% Allowed percent for each variable
allowedPercent = [0.066;   % depth
                  0.033;   % vertical velocity
                  0.066;   % pitch
                  0.066;   % roll
                  0.066;   % heading
                  0.033;   % pitch rate
                  0.033;   % roll rate
                  0.033];  % heading rate

allowedPercentLabel = ["6.6%";
                       "3.3%";
                       "6.6%";
                       "6.6%";
                       "6.6%";
                       "3.3%";
                       "3.3%";
                       "3.3%"];

% Preallocate result arrays
nVars = numel(varNames);

Variable = strings(nVars,1);
Units = strings(nVars,1);
Allowed_Percent = strings(nVars,1);

Mission_Min = zeros(nVars,1);
Mission_Max = zeros(nVars,1);
Mission_Range = zeros(nVars,1);
E_allow = zeros(nVars,1);

Max_E_actual = zeros(nVars,1);
Min_E_actual = zeros(nVars,1);
E_actual_range = zeros(nVars,1);
Std_E_actual = zeros(nVars,1);

Range_Result = strings(nVars,1);
Std_Result = strings(nVars,1);
Overall_Result = strings(nVars,1);

for i = 1:nVars

    var_mission = missionData{i};
    var_sim     = simData{i};

    % Actual error:
    % e_actual = |Var_mission - Var_simulation|
    e_actual = abs(var_mission - var_sim);

    % Allowed error:
    % E_allow = (max(var_mission) - min(var_mission)) * allowed percent
    mission_min = min(var_mission);
    mission_max = max(var_mission);
    mission_range = mission_max - mission_min;
    allowable_error = mission_range * allowedPercent(i);

    % Requirement check #1:
    % max(e_actual) - min(e_actual) < E_allow
    actual_error_range = max(e_actual) - min(e_actual);
    range_pass = actual_error_range < allowable_error;

    % Requirement check #2:
    % std(e_actual) < E_allow
    actual_error_std = std(e_actual);
    std_pass = actual_error_std < allowable_error;

    % Overall result: both checks must pass
    overall_pass = range_pass && std_pass;

    % Store results
    Variable(i) = varNames(i);
    Units(i) = varUnits(i);
    Allowed_Percent(i) = allowedPercentLabel(i);

    Mission_Min(i) = mission_min;
    Mission_Max(i) = mission_max;
    Mission_Range(i) = mission_range;
    E_allow(i) = allowable_error;

    Max_E_actual(i) = max(e_actual);
    Min_E_actual(i) = min(e_actual);
    E_actual_range(i) = actual_error_range;
    Std_E_actual(i) = actual_error_std;

    if range_pass
        Range_Result(i) = "PASS";
    else
        Range_Result(i) = "FAIL";
    end

    if std_pass
        Std_Result(i) = "PASS";
    else
        Std_Result(i) = "FAIL";
    end

    if overall_pass
        Overall_Result(i) = "PASS";
    else
        Overall_Result(i) = "FAIL";
    end

end

% Create results table
% error_results = table(Variable, Units, Allowed_Percent, ...
                      %Mission_Min, Mission_Max, Mission_Range, E_allow, ...
                      %Max_E_actual, Min_E_actual, E_actual_range, Std_E_actual, ...
                      %Range_Result, Std_Result, Overall_Result);

error_results = table(Variable, Units, Allowed_Percent, ...
                      E_allow, ...
                      E_actual_range, Std_E_actual, ...
                      Range_Result, Std_Result);

disp(' ');
disp('Error Evaluation Results:');
disp(error_results);

% Save results
writetable(error_results, 'dynamics_error_evaluation.csv');

%% Allowed Error Envelope Values for Plots

% 6.6%: depth, pitch, roll, heading
% 3.3%: vertical velocity, pitch rate, roll rate, heading rate

Eallow_depth = (max(eng_depth) - min(eng_depth)) * 0.066;
Eallow_w     = (max(eng_w) - min(eng_w)) * 0.033;

Eallow_pitch   = (max(eng_pitchang) - min(eng_pitchang)) * 0.066;
Eallow_roll    = (max(eng_rollang) - min(eng_rollang)) * 0.066;
Eallow_heading = (max(eng_heading_unwrapped) - min(eng_heading_unwrapped)) * 0.066;

Eallow_pitch_rate   = (max(eng_thetadot) - min(eng_thetadot)) * 0.033;
Eallow_roll_rate    = (max(eng_phidot) - min(eng_phidot)) * 0.033;
Eallow_heading_rate = (max(eng_headingdot) - min(eng_headingdot)) * 0.033;


%% Plot angular rates with allowed error envelopes

figure('Name','Angular rates vs mission data');

subplot(3,1,1)
plot(t, X(:,2), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_thetadot, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_thetadot + Eallow_pitch_rate, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_thetadot - Eallow_pitch_rate, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
ylabel('\theta dot (deg/s)')
xlabel('Time since start test point, s')
title('\theta dot vs time')
legend('Modeled \theta dot', ...
       'Mission \theta dot', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')
for k = 1:numel(test_t)
    xline(test_t(k), ':', 'HandleVisibility','off');
end

subplot(3,1,2)
plot(t, X(:,1), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_phidot, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_phidot + Eallow_roll_rate, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_phidot - Eallow_roll_rate, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
ylabel('\phi dot (deg/s)')
xlabel('Time since start test point, s')
title('\phi dot vs time')
legend('Modeled \phi dot', ...
       'Mission \phi dot', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')
for k = 1:numel(test_t)
    xline(test_t(k), ':', 'HandleVisibility','off');
end

subplot(3,1,3)
plot(t, X(:,3), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_headingdot, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_headingdot + Eallow_heading_rate, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_headingdot - Eallow_heading_rate, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
xlabel('Time since start test point, s')
ylabel('Heading dot (deg/s)')
title('Heading dot vs time')
legend('Modeled heading dot', ...
       'Mission heading dot', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')
for k = 1:numel(test_t)
    xline(test_t(k), ':', 'HandleVisibility','off');
end


%% Plot Euler angles with allowed error envelopes

figure('Name','Euler angles vs mission data');

subplot(3,1,1)
plot(t, X(:,5), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_pitchang, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_pitchang + Eallow_pitch, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_pitchang - Eallow_pitch, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
ylabel('\theta / pitch (deg)')
xlabel('Time since start test point, s')
title('\theta vs time')
legend('Modeled \theta', ...
       'Mission \theta', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')

subplot(3,1,2)
plot(t, X(:,4), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_rollang, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_rollang + Eallow_roll, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_rollang - Eallow_roll, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
ylabel('\phi / roll (deg)')
xlabel('Time since start test point, s')
title('\phi vs time')
legend('Modeled \phi', ...
       'Mission \phi', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')

subplot(3,1,3)
plot(t, X(:,6), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_head, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_head + Eallow_heading, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_head - Eallow_heading, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
xlabel('Time since start test point, s')
ylabel('Heading (deg)')
title('Heading vs time')
legend('Modeled heading', ...
       'Mission heading', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')


%% Plot depth and vertical velocity with allowed error envelopes

figure('Name','Depth and vertical velocity vs mission data');

subplot(2,1,1)
plot(t, X(:,12), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_depth, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_depth + Eallow_depth, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_depth - Eallow_depth, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
set(gca, 'YDir', 'reverse')   % Makes depth increase downward
ylabel('Depth (m)')
xlabel('Time since start test point, s')
title('Depth vs time')
legend('Modeled depth', ...
       'Mission depth', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')

subplot(2,1,2)
plot(t, X(:,9), 'r-', 'LineWidth', 1.2); hold on
plot(test_t, eng_w, 'bo-', 'LineWidth', 1.2)
plot(test_t, eng_w + Eallow_w, 'k--', 'LineWidth', 1.0)
plot(test_t, eng_w - Eallow_w, 'k--', 'LineWidth', 1.0)
grid on
xlim([0 dt])
xlabel('Time since start test point, s')
ylabel('Vertical velocity, w (m/s)')
title('Vertical velocity vs time')
legend('Modeled w', ...
       'Mission w', ...
       'Mission + E_{allow}', ...
       'Mission - E_{allow}', ...
       'Location','best')