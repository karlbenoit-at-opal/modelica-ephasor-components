within OpalRT.GenUnits.WT4G1;
class WT4G1
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // system parameters
  parameter Integer IBUS = 100 "Located system bus";
  parameter String ID = "M1" "Machine Identifier";
  parameter Real SB = 1000 "Machine base power (MVA)";
  parameter Real fn = 60 "FReqnuency(HZ)";
  parameter Real Vt_abs = 1.03 "initial voltage in p.u.";
  parameter Real Vt_ang = -10.6 "initial bus anglein degree";
  parameter Real P_gen = 600 "initial P in Mvar.";
  parameter Real Q_gen = 93.6 "initial Q in Mvar.";
  // Model parameters
  parameter Real TIQCmd = 0.02 "Converter time constant for IQcmd, second";
  parameter Real TIPCmd = 0.02 " Converter time constant for IPcmd, second";
  parameter Real VLVPL1 = 0.4 "Low Voltage power Logic (LVPL), voltage 1 (pu)";
  parameter Real VLVPL2 = 0.9 "LVPL voltage 2 (pu)";
  parameter Real GLVPL = 1.11 "LVPL gain";
  parameter Real VHVRCR = 1.2 "High Voltage reactive Current (HVRC) logic,voltage (pu)";
  parameter Real CURHVRCR = 2.0 "HVRC logic, current (pu)";
  parameter Real RIp_LVPL = 2.0 "Rate of active current change";
  parameter Real T_LVPL = 0.02 "Voltage sensor for LVPL, second";
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Positive_sequence = 1 "1: positive sequence, 0: Unbalanced 3-phase" annotation(Dialog(tab = "3-phase network parameters"));
  parameter Real Vbase = 1 "Base Voltage of connected bus, used for 3-phase network" annotation(Dialog(tab = "3-phase network parameters"));
  parameter Real Sbase_network = 1 "Network base power, used for 3-phase network" annotation(Dialog(tab = "3-phase network parameters"));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Load.ConstantImpedance2 rx_load1(R = ZSOURCE_RE, X = 999999) annotation(Placement(visible = true, transformation(origin = {80, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Renewable.WindTurbineGenerator.WT4.WT4G1 wt4g1_base1(SB = SB, fn = fn, Vt_abs = Vt_abs, Vt_ang = Vt_ang, P_gen = P_gen, Q_gen = Q_gen, TIQCmd = TIQCmd, TIPCmd = TIPCmd, VLVPL1 = VLVPL1, VLVPL2 = VLVPL2, GLVPL = GLVPL, VHVRCR = VHVRCR, CURHVRCR = CURHVRCR, RIp_LVPL = RIp_LVPL, T_LVPL = T_LVPL) annotation(Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Auxiliary.BaseConversion machine2net_base_change1(Sb_n = 1, Sb_p = if Positive_sequence == 1 then 1 else Sbase_network * 1e6, Vb_n = 1, Vb_p = if Positive_sequence == 1 then 1 else Vbase) annotation(Placement(visible = true, transformation(origin = {40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(machine2net_base_change1.n, bus0) annotation(Line(points = {{50, -46}, {73.613, -46}, {73.613, -46.5494}, {73.613, -46.5494}}));
  connect(rx_load1.p, machine2net_base_change1.p) annotation(Line(points = {{70, -80}, {29.4993, -80}, {29.4993, -46.0081}, {29.4993, -46.0081}}));
  connect(machine2net_base_change1.p, wt4g1_base1.p) annotation(Line(points = {{30, -46}, {10.2842, -46}, {10.2842, -32.4763}, {-4.05954, -32.4763}, {-4.05954, -32.4763}}));
  connect(wt4g1_base1.Ipcmd00, wt4g1_base1.IPCMD) annotation(Line(points = {{-20, -5}, {-20.2733, -5}, {-20.2733, 5.01139}, {-45.7859, 5.01139}, {-45.7859, -23.0068}, {-35.3075, -23.0068}, {-35.3075, -23.0068}}, color = {0, 0, 127}));
  connect(wt4g1_base1.Iqcmd00, wt4g1_base1.IQCMD) annotation(Line(points = {{-29, -5}, {-29.6128, -5}, {-29.6128, 2.2779}, {-40.0911, 2.2779}, {-40.0911, -13.2118}, {-36.9021, -13.2118}, {-36.9021, -13.2118}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {0.625, 0}, extent = {{-99.6875, 99.6875}, {99.6875, -99.6875}})}));
end WT4G1;
