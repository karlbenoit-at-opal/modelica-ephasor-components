within OpalRT.GenUnits.GENROU;
class GENROU_ESST1A_PSS2B
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1000 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 100 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 0.95 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -2 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1200 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 60 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 10.2 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.5 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 1.02 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 8.2 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 3 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.5 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.5231 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.361 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.41 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.2 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.5 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.6 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // ESST1A parameters
  parameter Real TR_ex = 0.02 "(sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VIMAX_ex = 10 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VIMIN_ex = -10 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real TC_ex = 1 "(sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real TB_ex = 1 "(sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real TC1_ex = 0 "(sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real TB1_ex = 0 "(sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real KA_ex = 210 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real TA_ex = 0 "(sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VAMAX_ex = 10 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VAMIN_ex = -10 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VRMAX_ex = 6.43 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VRMIN_ex = -6 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real KC_ex = 0.038 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real KF_ex = 0 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real TF_ex = 0 "> 0 (sec)" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real KLR_ex = 4.54 annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real ILR_ex = 4.4 annotation(Dialog(tab = "ESST1A Parameters"));
  // ICONS
  parameter Real UEL_ex = 1 "1,2 or 3" annotation(Dialog(tab = "ESST1A Parameters"));
  parameter Real VOS_ex = 1 "1 or 2" annotation(Dialog(tab = "ESST1A Parameters"));
  // PSS2B Parameters
  parameter Real TW1_pss = 2 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real TW2_pss = 2 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T6_pss = 0.05 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real TW3_pss = 2 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real TW4_pss = 1.5 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T7_pss = 2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real KS2_pss = 0.259 "T7/(2*H)" annotation(Dialog(tab = "PSS2B Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T8_pss = 0.5 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T9_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real KS1_pss = 15 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T1_pss = 0.15 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T2_pss = 0.05 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T3_pss = 0.15 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T4_pss = 0.05 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VSTMAX_pss = 0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VSTMIN_pss = -0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS1MAX_pss = 0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS1MIN_pss = -0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS2MAX_pss = 0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS2MIN_pss = -0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T10_pss = 0.3 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T11_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  /// PSS2B ICONs
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M1_pss = 0 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M2_pss = 3 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M3_pss = 0 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M4_pss = 5 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M5_pss = 1 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.Stabilizer.PSS2B pss2b1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID, T10 = T10_pss, T11 = T11_pss, VS1MAX = VS1MAX_pss, VS1MIN = VS1MIN_pss, VS2MAX = VS2MAX_pss, VS2MIN = VS2MIN_pss) annotation(Placement(visible = true, transformation(origin = {-48, 22}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {36, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {36, 18}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST1A esst1a1(UEL = UEL_ex, VOS = VOS_ex, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, TC1 = TC1_ex, TB1 = TB1_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex, KLR = KLR_ex, ILR = ILR_ex) annotation(Placement(visible = true, transformation(origin = {4, 28}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {96, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-35, 43}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-34, 58}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = if UEL_ex <> 1 then -Modelica.Constants.inf else 0;
equation
  connect(dVREF, esst1a1.dVREF) annotation(Line(points = {{-40, -20}, {-16.3079, -20}, {-16.3079, 20.6566}, {-8.26265, 20.6566}, {-8.26265, 20.6566}}));
  connect(genrou1.VI, pss2b1.VI2) annotation(Line(points = {{48.5, 18}, {56.8834, 18}, {56.8834, -6.98909}, {-67.5612, -6.98909}, {-67.5612, 18.0551}, {-62.7076, 18.0551}, {-62.7076, 18.0551}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, pss2b1.PSS_AUX2[2]) annotation(Line(points = {{48.5, 14.75}, {54.942, 14.75}, {54.942, -3.88283}, {-70.085, -3.88283}, {-70.085, 15.9196}, {-62.7076, 15.9196}, {-62.7076, 15.9196}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2b1.PSS_AUX2[1]) annotation(Line(points = {{48.5, 8}, {-70.085, 8}, {-70.085, 15.9196}, {-63.4842, 15.9196}, {-63.4842, 15.9196}}, color = {0, 0, 127}));
  connect(pss2b1.VI2, pss2b1.VI) annotation(Line(points = {{-63, 18}, {-67.4294, 18}, {-67.4294, 24.0943}, {-63.4426, 24.0943}, {-63.4426, 24.0943}}, color = {0, 0, 127}));
  connect(genrou1.VI, esst1a1.VI) annotation(Line(points = {{48.5, 18}, {56.8834, 18}, {56.8834, 35.5279}, {17.4727, 35.5279}, {17.4727, 35.5279}}, color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{36, 5.5}, {36.3367, 5.5}, {36.3367, -22.481}, {96, -22.481}, {96, -22}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{36, 68}, {36.0802, 68}, {36.0802, 30.5}, {36, 30.5}}));
  connect(esst1a1.EFD0, genrou1.EFD0) annotation(Line(points = {{16.5, 18}, {23.25, 18}}, color = {0, 0, 127}));
  connect(esst1a1.EFD, genrou1.EFD) annotation(Line(points = {{16.5, 20.5}, {20.25, 20.5}, {20.25, 21.5}, {23.5, 21.5}}, color = {0, 0, 127}));
  connect(esst1a1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{16.5, 23}, {20.25, 23}, {20.25, 24.25}, {23.25, 24.25}}, color = {0, 0, 127}));
  connect(esst1a1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{16.5, 26.25}, {20.25, 26.25}, {20.25, 28}, {23.25, 28}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation(Line(points = {{23.5, 8}, {20, 8}, {20, 10.5}, {23.5, 10.5}}, color = {0, 0, 127}));
  connect(esst1a1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-8.5, 34.25}, {-14, 34.25}, {-14, 48}, {56, 48}, {56, 24.25}, {48.5, 24.25}}, color = {0, 0, 127}));
  connect(esst1a1.VOEL, const.y) annotation(Line(points = {{-8.5, 27}, {-26, 27}, {-26, 43}, {-29.5, 43}}, color = {0, 0, 127}));
  connect(constant1.y, esst1a1.VUEL) annotation(Line(points = {{-28.5, 58}, {-22, 58}, {-22, 30.5}, {-8.5, 30.5}}, color = {0, 0, 127}));
  connect(pss2b1.VOTHSG, esst1a1.VOTHSG) annotation(Line(points = {{-33, 16}, {-22, 16}, {-22, 23.5}, {-8.5, 23.5}}, color = {0, 0, 127}));
  connect(pss2b1.PSS_AUX, pss2b1.PSS_AUX2) annotation(Line(points = {{-63, 22}, {-70, 22}, {-70, 16}, {-62.7, 16}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_ESST1A_PSS2B"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN"), Text(origin = {-68.1086, 61.7315}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "dGREF"), Text(origin = {-73.8448, -60.1801}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "TRIP")}));
end GENROU_ESST1A_PSS2B;
