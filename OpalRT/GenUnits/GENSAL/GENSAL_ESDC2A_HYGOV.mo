within OpalRT.GenUnits.GENSAL;
class GENSAL_ESDC2A_HYGOV
  parameter Real partType = 1;
  //GENSAL
  parameter Integer IBUS = 100 "Located system bus";
  parameter String ID = "M1" "Machine Identifier";
  parameter Real P_gen = 1100 "Bus Active Power, MW";
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR";
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u.";
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg.";
  parameter Real SB = 1000 "Machine Base Power, MVA";
  parameter Real fn = 50 "Nominal frequency";
  parameter Real ZSOURCE_RE = 0 "Machine source impedence";
  parameter Real Tdo_p = 7 "d-axis transient time constant";
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s";
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s";
  parameter Real H = 50 "Inertia constant";
  parameter Real D = 0 "Speed damping";
  parameter Real Xd = 0.2 "d-axis reactance, p.u.";
  parameter Real Xq = 0.19 "q-axis reactance, p.u.";
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u.";
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u.";
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u.";
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input";
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input";
  parameter Real ZSOURCE_IM = Xd_s "Machine source impedence";
  //ESDC2A
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESDC1A parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESDC1A parameters"));
  //HYGOV
  parameter Real R_tg = 0.06 "Permanent Droop";
  parameter Real r_tg = 0.4 "Temporary Droop";
  parameter Real Tr_tg = 8 "(>0) Governor time constant";
  parameter Real Tf_tg = 0.05 "(>0) Filter time constant";
  parameter Real Tg_tg = 0.2 "(>0) Servo time constant";
  parameter Real VELM_tg = 0.01 "Gate velocity limit";
  parameter Real GMAX_tg = 0.601 "Maximum gate limit";
  parameter Real GMIN_tg = 0 "Minimum gate limit";
  parameter Real TW_tg = 1.2 "(>0) Water time constant";
  parameter Real At_tg = 2.5 "Trubine gain";
  parameter Real Dturb_tg = 0 "Turbine damping";
  parameter Real qNL_tg = 0.5 "No power flow";
  OpalRT.Electrical.Control.Excitation.ESDC2A esdc2a1(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, TE = TE_ex) annotation(Placement(visible = true, transformation(origin = {-9, 7}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12, ZSOURCE_RE = ZSOURCE_RE) annotation(Placement(visible = true, transformation(origin = {30, -6}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-54, 18}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {42, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {80, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin Trip annotation(Placement(visible = true, transformation(origin = {25, 59}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV hygov1(R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, VELM = VELM_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, At = At_tg, Dturb = Dturb_tg, qNL = qNL_tg) annotation(Placement(visible = true, transformation(origin = {-6, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-56, -22}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real noVUEL(fixed = false, start = 1);
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-54, 34}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
initial equation
  noVUEL = -Modelica.Constants.inf;
equation
  connect(dVREF, esdc2a1.dVREF) annotation(Line(points = {{-60, 0}, {-31.9634, 0}, {-31.9634, -2.17438}, {-23.9182, -2.17438}, {-23.9182, -2.17438}}));
  connect(esdc2a1.VI, gensal1.VI) annotation(Line(points = {{6, 16}, {60.013, 16}, {60.013, -6.08827}, {45.8795, -6.08827}, {45.8795, -6.08827}}, color = {0, 0, 127}));
  connect(gensal1.MBASE, hygov1.MBASE) annotation(Line(points = {{45, -14.1}, {59.3606, -14.1}, {59.3606, -63.7094}, {-20.4392, -63.7094}, {-20.4392, -26.5275}, {-16.3079, -26.5275}, {-16.3079, -26.5275}}, color = {0, 0, 127}));
  connect(dGREF, hygov1.dGREF) annotation(Line(points = {{-56, -22}, {-31.9634, -22}, {-31.9634, -14.3509}, {-16.0904, -14.3509}, {-16.0904, -14.3509}}));
  connect(gensal1.VI, hygov1.VI) annotation(Line(points = {{45, -6}, {60.013, -6}, {60.013, -65.6664}, {-21.0915, -65.6664}, {-21.0915, -22.1787}, {-15.873, -22.1787}, {-15.873, -22.1787}}, color = {0, 0, 127}));
  connect(gensal1.PMECH0, hygov1.PMECH0) annotation(Line(points = {{15, -18}, {10.2196, -18}, {10.2196, -16.3079}, {4.13133, -16.3079}, {4.13133, -16.3079}}, color = {0, 0, 127}));
  connect(Trip, gensal1.TRIP) annotation(Line(points = {{25, 59}, {29.7433, 59}, {29.7433, 9}, {30, 9}}));
  connect(bus0, gensal1.p) annotation(Line(points = {{42, -48}, {29.2015, -48}, {29.2015, -21}, {30, -21}}));
  connect(gensal1.EFD0, esdc2a1.EFD0) annotation(Line(points = {{14.7, -6}, {10, -6}, {10, -5}, {6, -5}}, color = {0, 0, 127}));
  connect(esdc2a1.EFD, gensal1.EFD) annotation(Line(points = {{6, -2}, {10, -2}, {10, -1.8}, {15, -1.8}}, color = {0, 0, 127}));
  connect(esdc2a1.ETERM0, gensal1.ETERM0) annotation(Line(points = {{6, 1}, {10, 1}, {10, 1.5}, {14.7, 1.5}}, color = {0, 0, 127}));
  connect(esdc2a1.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{6, 4.9}, {10, 4.9}, {10, 6}, {14.7, 6}}, color = {0, 0, 127}));
  connect(hygov1.PMECH, gensal1.PMECH) annotation(Line(points = {{4, -14}, {6, -14}, {6, -15}, {15, -15}}, color = {0, 0, 127}));
  connect(esdc2a1.XADIFD, gensal1.XADIFD) annotation(Line(points = {{-24, 14.5}, {-30, 14.5}, {-30, 28}, {52, 28}, {52, 1.5}, {45, 1.5}}, color = {0, 0, 127}));
  connect(constant1.y, esdc2a1.VUEL) annotation(Line(points = {{-48.5, 34}, {-34, 34}, {-34, 10}, {-24, 10}}, color = {0, 0, 127}));
  connect(esdc2a1.VOEL, const.y) annotation(Line(points = {{-24, 5.8}, {-38, 5.8}, {-38, 18}, {-48.5, 18}}, color = {0, 0, 127}));
  connect(esdc2a1.VOTHSG, const.y) annotation(Line(points = {{-24, 1.6}, {-34, 1.6}, {-34, 6}, {-38, 5.8}, {-38, 18}, {-48.5, 18}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, hygov1.SLIP) annotation(Line(points = {{45, -18}, {58, -18}, {58, -62}, {-20, -62}, {-20, -30}, {-16, -30}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-95.89, -22.34}, extent = {{8.6, 26.96}, {181.13, -7.96}}, textString = "GENROU_ESDC2A_HYGOV"), Rectangle(origin = {2.05392, -19.5122}, extent = {{-99.6149, 78.819}, {99.6149, -78.819}})}));
end GENSAL_ESDC2A_HYGOV;
