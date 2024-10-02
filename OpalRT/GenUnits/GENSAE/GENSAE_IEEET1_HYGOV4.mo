within OpalRT.GenUnits.GENSAE;
class GENSAE_IEEET1_HYGOV4
  parameter Real partType = 1;
  // GENSAE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAE"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAE"));
  parameter Real Tdo_p = 10 "d-axis transient time constant" annotation(Dialog(tab = "GENSAE"));
  parameter Real Tdo_s = 0.05 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAE"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAE"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAE"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAE"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAE"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAE"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAE"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAE"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAE"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAE"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAE"));
  // IEEET1 Parameters
  parameter Real TR_ex = 0.025 "(sec)" annotation(Dialog(tab = "IEEET1"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEET1"));
  parameter Real TA_ex = 0.2 "(sec)" annotation(Dialog(tab = "IEEET1"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "IEEET1"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEET1"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "IEEET1"));
  parameter Real TE_ex = 0.35 "(>0) (sec)" annotation(Dialog(tab = "IEEET1"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEET1"));
  parameter Real TF_ex = 0.4 "(>0) (sec)" annotation(Dialog(tab = "IEEET1"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "IEEET1"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEET1"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEET1"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEET1"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEET1"));
  //HYGOV4 Parameters
  parameter Real db_tg = 0.06 "Hysteresis deadband" annotation(Dialog(tab = "HYGOV4"));
  parameter Real R_tg = 0.06 "Permanent Droop" annotation(Dialog(tab = "HYGOV4"));
  parameter Real r_tg = 0.4 "Temporary Droop" annotation(Dialog(tab = "HYGOV4"));
  parameter Real Tr_tg = 8 "(>0) Dashpot time constant" annotation(Dialog(tab = "HYGOV4"));
  parameter Real Tf_tg = 0.05 "(>0) Pilot valve time constant" annotation(Dialog(tab = "HYGOV4"));
  parameter Real Tg_tg = 0.2 "(>0) Actuator time const" annotation(Dialog(tab = "HYGOV4"));
  parameter Real Uopen_tg = 0.07 "Rate of gate Opening" annotation(Dialog(tab = "HYGOV4"));
  parameter Real Uclose_tg = -0.11 "(<0) Rate of gate clsoing" annotation(Dialog(tab = "HYGOV4"));
  parameter Real GMAX_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real GMIN_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real Dturb_tg = 0.01 annotation(Dialog(tab = "HYGOV4"));
  parameter Real TW_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real G0_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real Q0_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real G1_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real Q1_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real G2_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real Q2_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real G3_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real Q3_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real G4_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real Q4_tg = 1.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F0_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P0_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F1_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P1_tg = 0.0 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F2_tg = 0.1 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P2_tg = 0.1 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F3_tg = 0.2 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P3_tg = 0.1 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F4_tg = 0.3 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P4_tg = 0.3 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F5_tg = 0.4 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P5_tg = 0.4 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F6_tg = 0.5 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P6_tg = 0.5 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F7_tg = 0.6 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P7_tg = 0.6 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F8_tg = 0.7 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P8_tg = 0.7 annotation(Dialog(tab = "HYGOV4"));
  parameter Real F9_tg = 0.8 annotation(Dialog(tab = "HYGOV4"));
  parameter Real P9_tg = 0.8 annotation(Dialog(tab = "HYGOV4"));
  OpalRT.Electrical.Control.Excitation.IEEET1 ieeet11(ID = M_ID, TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-28, 32}, extent = {{-30.375, -30.375}, {30.375, 30.375}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-86, 38}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV4 hygov41(db = db_tg, R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, Uopen = Uopen_tg, Uclose = Uclose_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, Dturb = Dturb_tg, G0 = G0_tg, G1 = G1_tg, G2 = G2_tg, G3 = G3_tg, G4 = G4_tg, Q0 = Q0_tg, Q1 = Q1_tg, Q2 = Q2_tg, Q3 = Q3_tg, Q4 = Q4_tg, F0 = F0_tg, F1 = F1_tg, F2 = F2_tg, F3 = F3_tg, F4 = F4_tg, F5 = F5_tg, F6 = F6_tg, F7 = F7_tg, F8 = F8_tg, F9 = F9_tg, P0 = P0_tg, P1 = P1_tg, P2 = P2_tg, P3 = P3_tg, P4 = P4_tg, P5 = P5_tg, P6 = P6_tg, P7 = P7_tg, P8 = P8_tg, P9 = P9_tg) annotation(Placement(visible = true, transformation(origin = {-24, -32}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {46, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, -58}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-102, -6}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0), iconTransformation(origin = {-94, -60}, extent = {{-13.9375, -13.9375}, {13.9375, 13.9375}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-102, 16}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-94, 72}, extent = {{-13.5, -13.5}, {13.5, 13.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {44, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {94, 70}, extent = {{15, -15}, {-15, 15}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAE gensae1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {46, 6}, extent = {{-29, -29}, {29, 29}}, rotation = 0)));
equation
  connect(dVREF, ieeet11.dVREF) annotation(Line(points = {{-102, 16}, {-71.6738, 16}, {-71.6738, 13.3047}, {-56.6524, 13.3047}, {-56.6524, 13.3047}}));
  connect(dGREF, hygov41.dGREF) annotation(Line(points = {{-102, -6}, {-79.8283, -6}, {-79.8283, -12.0172}, {-50.2146, -12.0172}, {-50.2146, -12.0172}}));
  connect(gensae1.VI, hygov41.VI) annotation(Line(points = {{75, 6}, {94.4206, 6}, {94.4206, -82.8326}, {-66.5236, -82.8326}, {-66.5236, -32.1888}, {-49.7854, -32.1888}, {-49.7854, -32.1888}}, color = {0, 0, 127}));
  connect(gensae1.MBASE, hygov41.MBASE) annotation(Line(points = {{75, -9.66}, {87.5536, -9.66}, {87.5536, -78.1116}, {-61.8026, -78.1116}, {-61.8026, -42.0601}, {-49.7854, -42.0601}, {-49.7854, -42.0601}}, color = {0, 0, 127}));
  connect(ieeet11.VI, gensae1.VI) annotation(Line(points = {{2.375, 50.225}, {87.1245, 50.225}, {87.1245, 6.00858}, {75.9657, 6.00858}, {75.9657, 6.00858}}, color = {0, 0, 127}));
  connect(TRIP, gensae1.TRIP) annotation(Line(points = {{44, 64}, {46.4789, 64}, {46, 36.1502}, {46, 35}}));
  connect(gensae1.p, bus0) annotation(Line(points = {{46, -23}, {47.4178, -23}, {47.4178, -44.6009}, {47.4178, -44.6009}}));
  connect(gensae1.EFD0, ieeet11.EFD0) annotation(Line(points = {{16.42, 6}, {9.38967, 6}, {9.38967, 7.51174}, {3.75587, 7.51174}, {3.75587, 6.57277}, {3.75587, 6.57277}}, color = {0, 0, 127}));
  connect(ieeet11.EFD, gensae1.EFD) annotation(Line(points = {{2.375, 13.775}, {14.554, 13.775}, {17, 14.554}, {17, 14.12}}, color = {0, 0, 127}));
  connect(gensae1.ETERM0, ieeet11.ETERM0) annotation(Line(points = {{16.42, 20.5}, {4.69484, 20.5}, {4.69484, 20.6573}, {4.69484, 20.6573}}, color = {0, 0, 127}));
  connect(gensae1.EX_AUX, ieeet11.EX_AUX) annotation(Line(points = {{16.42, 29.2}, {9.38967, 29.2}, {9.38967, 27.23}, {3.75587, 27.23}, {3.75587, 27.23}}, color = {0, 0, 127}));
  connect(gensae1.XADIFD, ieeet11.XADIFD) annotation(Line(points = {{75, 20.5}, {87.3239, 20.5}, {87.3239, 83.0986}, {-68.5446, 83.0986}, {-68.5446, 46.9484}, {-59.1549, 46.9484}, {-59.1549, 46.9484}}, color = {0, 0, 127}));
  connect(hygov41.PMECH, gensae1.PMECH) annotation(Line(points = {{2, -11.2}, {15.493, -11.2}, {17, -11.4}, {17, -11.4}}, color = {0, 0, 127}));
  connect(gensae1.PMECH0, hygov41.PMECH0) annotation(Line(points = {{17, -17.2}, {0, -17.2}, {2, -16.4}, {2, -16.4}}, color = {0, 0, 127}));
  connect(gensae1.SLIP, hygov41.SLIP) annotation(Line(points = {{75, -17.2}, {82.6291, -17.2}, {82.6291, -66.6667}, {-59.6244, -66.6667}, {-59.6244, -53.5211}, {-54, -52.8}, {-50, -52.8}}, color = {0, 0, 127}));
  connect(const.y, ieeet11.VOTHSG) annotation(Line(points = {{-80.5, 38}, {-65.7277, 38}, {-65.7277, 20.1878}, {-59.6244, 20.1878}, {-59.6244, 20.1878}}, color = {0, 0, 127}));
  connect(const.y, ieeet11.VOEL) annotation(Line(points = {{-80.5, 38}, {-65.7277, 38}, {-65.7277, 28.6385}, {-60.0939, 28.6385}, {-60.0939, 28.6385}}, color = {0, 0, 127}));
  connect(const.y, ieeet11.VUEL) annotation(Line(points = {{-80.5, 38}, {-59.1549, 38}, {-59.1549, 37.5587}, {-59.1549, 37.5587}}, color = {0, 0, 127}));
  connect(ieeet11.VOTHSG, ieeet11.VOEL) annotation(Line(points = {{-58.375, 21.065}, {-58.375, 21.065}, {-58.375, 39.895}, {-50.375, 39.895}}, color = {0, 0, 127}));
  connect(ieeet11.VOEL, ieeet11.VUEL) annotation(Line(points = {{-58.375, 29.57}, {-58.375, 29.57}, {-58.375, 44.875}, {-50.375, 45.075}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {2.8169, 0}, extent = {{-100, 97.6526}, {95.3052, -96.2441}}), Text(origin = {-24.6573, 20.3099}, extent = {{-58, 29}, {107.296, -52.0047}}, textString = "GENSAE_IEEET1_HYGOV4"), Text(origin = {69.061, -58.8732}, extent = {{-30, 14}, {20.1408, -9.77465}}, textString = "PIN"), Text(origin = {-52.8171, 70.8969}, extent = {{-24.18, 13.15}, {24.18, -13.15}}, textString = "dVREF"), Text(origin = {-61.9676, -53.519}, extent = {{-15.02, 14.55}, {32.3909, -28.165}}, textString = "dGREF"), Text(origin = {57.9771, 70.6564}, extent = {{-21.36, 12.91}, {18.0736, -10.5626}}, textString = "TRIP")}));
end GENSAE_IEEET1_HYGOV4;
