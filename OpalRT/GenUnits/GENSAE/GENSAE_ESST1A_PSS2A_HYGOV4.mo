within OpalRT.GenUnits.GENSAE;
class GENSAE_ESST1A_PSS2A_HYGOV4
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
  // ESST1A parameters
  parameter Real TR_ex = 0.02 "(sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real VIMAX_ex = 10 annotation(Dialog(tab = "ESST1A"));
  parameter Real VIMIN_ex = -10 annotation(Dialog(tab = "ESST1A"));
  parameter Real TC_ex = 1 "(sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real TB_ex = 1 "(sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real TC1_ex = 0 "(sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real TB1_ex = 0 "(sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real KA_ex = 210 annotation(Dialog(tab = "ESST1A"));
  parameter Real TA_ex = 0 "(sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real VAMAX_ex = 10 annotation(Dialog(tab = "ESST1A"));
  parameter Real VAMIN_ex = -10 annotation(Dialog(tab = "ESST1A"));
  parameter Real VRMAX_ex = 6.43 annotation(Dialog(tab = "ESST1A"));
  parameter Real VRMIN_ex = -6 annotation(Dialog(tab = "ESST1A"));
  parameter Real KC_ex = 0.038 annotation(Dialog(tab = "ESST1A"));
  parameter Real KF_ex = 0 annotation(Dialog(tab = "ESST1A"));
  parameter Real TF_ex = 0 "> 0 (sec)" annotation(Dialog(tab = "ESST1A"));
  parameter Real KLR_ex = 4.54 annotation(Dialog(tab = "ESST1A"));
  parameter Real ILR_ex = 4.4 annotation(Dialog(tab = "ESST1A"));
  // ICONS
  parameter Real UEL_ex = 1 "1,2 or 3" annotation(Dialog(tab = "ESST1A"));
  parameter Real VOS_ex = 1 "1 or 2" annotation(Dialog(tab = "ESST1A"));
  // PSS2A Parameters
  parameter Real TW1_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2A"));
  parameter Real TW2_pss = 1 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T6_pss = 1 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real TW3_pss = 1.5 ">0" annotation(Dialog(tab = "PSS2A"));
  parameter Real TW4_pss = 0.2 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T7_pss = 0.2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real KS2_pss = 0.1 "T7/(2*H)" annotation(Dialog(tab = "PSS2A"));
  //T7/(2*H);
  parameter Real KS3_pss = 0.1 annotation(Dialog(tab = "PSS2A"));
  parameter Real T8_pss = 0.05 annotation(Dialog(tab = "PSS2A"));
  parameter Real T9_pss = 0.01 ">0" annotation(Dialog(tab = "PSS2A"));
  parameter Real KS1_pss = 0.15 annotation(Dialog(tab = "PSS2A"));
  parameter Real T1_pss = 0.1 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T2_pss = 0.1 annotation(Dialog(tab = "PSS2A"));
  parameter Real T3_pss = 0.01 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T4_pss = 1 annotation(Dialog(tab = "PSS2A"));
  parameter Real VSTMAX_pss = 1 annotation(Dialog(tab = "PSS2A"));
  parameter Real VSTMIN_pss = -1 annotation(Dialog(tab = "PSS2A"));
  // PSS2A ICONs
  parameter Real M0_pss = 2 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M1_pss = 2 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M2_pss = 2 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M3_pss = 2 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M4_pss = 1 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M5_pss = 2 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
  //HYGOV4 parameter
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
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {66, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAE gensae1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {52.5, 14.5}, extent = {{-26.5, -26.5}, {26.5, 26.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-68, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-70, 42}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-102, 40}, extent = {{-19.5, -13}, {19.5, 13}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV4 hygov41(db = db_tg, R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, Uopen = Uopen_tg, Uclose = Uclose_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, Dturb = Dturb_tg, G0 = G0_tg, G1 = G1_tg, G2 = G2_tg, G3 = G3_tg, G4 = G4_tg, Q0 = Q0_tg, Q1 = Q1_tg, Q2 = Q2_tg, Q3 = Q3_tg, Q4 = Q4_tg, F0 = F0_tg, F1 = F1_tg, F2 = F2_tg, F3 = F3_tg, F4 = F4_tg, F5 = F5_tg, F6 = F6_tg, F7 = F7_tg, F8 = F8_tg, F9 = F9_tg, P0 = P0_tg, P1 = P1_tg, P2 = P2_tg, P3 = P3_tg, P4 = P4_tg, P5 = P5_tg, P6 = P6_tg, P7 = P7_tg, P8 = P8_tg, P9 = P9_tg) annotation(Placement(visible = true, transformation(origin = {-10, -18}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-94, 12}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0), iconTransformation(origin = {-92, 70}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-92, -10}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {-92, -74}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST1A esst1a1(UEL = UEL_ex, VOS = VOS_ex, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, TC1 = TC1_ex, TB1 = TB1_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex, KLR = KLR_ex, ILR = ILR_ex) annotation(Placement(visible = true, transformation(origin = {-20, 40}, extent = {{-26.75, -26.75}, {26.75, 26.75}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {52, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {94, 72}, extent = {{12, 12}, {-12, -12}}, rotation = 0)));
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = if UEL_ex <> 1 then -Modelica.Constants.inf else 0;
equation
  connect(gensae1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{79, 7.61}, {93.9914, 7.61}, {93.9914, -63.0901}, {-126.18, -63.0901}, {-126.18, 31.3305}, {-121.459, 31.3305}, {-121.459, 31.3305}}, color = {0, 0, 127}));
  connect(gensae1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{79, -6.7}, {87.9828, -6.7}, {87.9828, -59.6567}, {-127.468, -59.6567}, {-127.468, 32.1888}, {-121.888, 32.1888}, {-121.888, 32.1888}}, color = {0, 0, 127}));
  connect(gensae1.VI, pss2a1.VI) annotation(Line(points = {{79, 14.5}, {96.1373, 14.5}, {96.1373, -65.6652}, {-130.901, -65.6652}, {-130.901, 42.9185}, {-121.888, 42.9185}, {-121.888, 42.9185}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points = {{-121.5, 32.2}, {-127.039, 32.2}, {-127.039, 39.0558}, {-121.03, 39.0558}, {-121.03, 39.0558}}, color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points = {{-121.5, 34.8}, {-130.901, 34.8}, {-130.901, 42.9185}, {-121.03, 42.9185}, {-121.03, 42.9185}}, color = {0, 0, 127}));
  connect(gensae1.MBASE, hygov41.MBASE) annotation(Line(points = {{79, 0.19}, {91.8455, 0.19}, {91.8455, -62.6609}, {-42.4893, -62.6609}, {-42.4893, -25.3219}, {-30.4721, -25.3219}, {-30.4721, -25.3219}}, color = {0, 0, 127}));
  connect(gensae1.VI, hygov41.VI) annotation(Line(points = {{79, 14.5}, {96.1373, 14.5}, {96.1373, -65.6652}, {-46.7811, -65.6652}, {-46.7811, -18.0258}, {-29.1845, -18.0258}, {-29.1845, -18.0258}}, color = {0, 0, 127}));
  connect(dGREF, hygov41.dGREF) annotation(Line(points = {{-92, -10}, {-56.6524, -10}, {-56.6524, -2.14592}, {-30.4721, -2.14592}, {-30.4721, -2.14592}}));
  connect(dVREF, esst1a1.dVREF) annotation(Line(points = {{-94, 12}, {-67.8112, 12}, {-67.8112, 24.0343}, {-47.2103, 24.0343}, {-47.2103, 24.0343}}));
  connect(esst1a1.VI, gensae1.VI) annotation(Line(points = {{6.75, 56.05}, {96.1373, 56.05}, {96.1373, 13.7339}, {81.1159, 13.7339}, {81.1159, 13.7339}}, color = {0, 0, 127}));
  connect(gensae1.EFD0, esst1a1.EFD0) annotation(Line(points = {{25.47, 14.5}, {12.6761, 14.5}, {6.75, 15.0235}, {6.75, 18.6}}, color = {0, 0, 127}));
  connect(esst1a1.EFD, gensae1.EFD) annotation(Line(points = {{6.75, 23.95}, {20.6573, 23.95}, {20.6573, 21.1268}, {25.8216, 21.1268}, {25.8216, 21.5962}, {25.8216, 21.5962}}, color = {0, 0, 127}));
  connect(gensae1.ETERM0, esst1a1.ETERM0) annotation(Line(points = {{25.47, 27.75}, {19.7183, 27.75}, {19.7183, 25.8216}, {14.554, 29.3}, {6.75, 29.3}}, color = {0, 0, 127}));
  connect(gensae1.EX_AUX, esst1a1.EX_AUX) annotation(Line(points = {{25.47, 35.7}, {20.6573, 35.7}, {20.6573, 31.4554}, {14.554, 36.255}, {6.75, 36.255}}, color = {0, 0, 127}));
  connect(gensae1.XADIFD, esst1a1.XADIFD) annotation(Line(points = {{79, 27.75}, {92.0188, 27.75}, {92.0188, 85.9155}, {-51.6432, 85.9155}, {-51.6432, 49.2958}, {-40.8451, 53.375}, {-46.75, 53.375}}, color = {0, 0, 127}));
  connect(constant1.y, esst1a1.VUEL) annotation(Line(points = {{-62.5, 60}, {-57.7465, 60}, {-57.7465, 42.2535}, {-39.9061, 45.35}, {-46.75, 45.35}}, color = {0, 0, 127}));
  connect(const.y, esst1a1.VOEL) annotation(Line(points = {{-64.5, 42}, {-61.0329, 42}, {-61.0329, 33.8028}, {-40.8451, 37.86}, {-46.75, 37.86}}, color = {0, 0, 127}));
  connect(pss2a1.VOTHSG, esst1a1.VOTHSG) annotation(Line(points = {{-82.5, 32.2}, {-74.1784, 32.2}, {-74.1784, 26.7606}, {-39.9061, 30.37}, {-46.75, 30.37}}, color = {0, 0, 127}));
  connect(gensae1.SLIP, hygov41.SLIP) annotation(Line(points = {{79, -6.7}, {87.7934, -6.7}, {87.7934, -59.6244}, {-39.9061, -59.6244}, {-39.9061, -33.8028}, {-29.5775, -33.8028}, {-29.5775, -33.8028}}, color = {0, 0, 127}));
  connect(gensae1.PMECH0, hygov41.PMECH0) annotation(Line(points = {{26, -6.7}, {11.2676, -6.7}, {11.2676, -6.57277}, {11.2676, -6.57277}}, color = {0, 0, 127}));
  connect(hygov41.PMECH, gensae1.PMECH) annotation(Line(points = {{10, -2}, {25.3521, -2}, {25.3521, -1.40845}, {25.3521, -1.40845}}, color = {0, 0, 127}));
  connect(gensae1.p, bus0) annotation(Line(points = {{52.5, -12}, {49.7653, -12}, {49.7653, -42.2535}, {66, -42.2535}, {66, -40}}));
  connect(TRIP, gensae1.TRIP) annotation(Line(points = {{52, 70}, {50.7042, 70}, {52.5, 41}, {52.5, 41}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {1.53052, -1.06103}, extent = {{-100, 100}, {97.1831, -97.1831}}), Text(origin = {-30.2911, 25.9437}, extent = {{-58, 29}, {110.113, -73.6009}}, textString = "GENSAE_ESST1A_PSS2A_HYGOV4"), Text(origin = {70, -80}, extent = {{-30, 14}, {30, -14}}, textString = "PIN"), Text(origin = {-61.0297, 72.5333}, extent = {{-17.37, 11.03}, {23.0038, -16.1943}}, textString = "dVREF"), Text(origin = {-58.9166, -70.1864}, extent = {{-17.14, 11.5}, {24.1823, -18.5423}}, textString = "dGREF"), Text(origin = {64.7856, 70.1924}, extent = {{-21.6, 13.85}, {14.0883, -8.2162}}, textString = "TRIP")}));
end GENSAE_ESST1A_PSS2A_HYGOV4;
