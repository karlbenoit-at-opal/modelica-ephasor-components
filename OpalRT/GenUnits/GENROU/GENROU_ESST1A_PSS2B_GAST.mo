within OpalRT.GenUnits.GENROU;
class GENROU_ESST1A_PSS2B_GAST
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
  // GAST Parameters
  parameter Real R_tg = 0.01 "Speed droop" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T1_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T2_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T3_tg = 0.3 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real AT_tg = 0.12 "Ambient temperature load limit" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real KT_tg = 0.2 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMAX_tg = 0.12 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMIN_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real DTURB_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
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
  OpalRT.Electrical.Control.Stabilizer.PSS2B pss2b1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID, T10 = T10_pss, T11 = T11_pss, VS1MAX = VS1MAX_pss, VS1MIN = VS1MIN_pss, VS2MAX = VS2MAX_pss, VS2MIN = VS2MIN_pss) annotation(Placement(visible = true, transformation(origin = {-60, 6}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {39, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-5, -21}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-42, 26}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {84, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {84, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {39, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST1A esst1a1(UEL = UEL_ex, VOS = VOS_ex, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, TC1 = TC1_ex, TB1 = TB1_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex, KLR = KLR_ex, ILR = ILR_ex) annotation(Placement(visible = true, transformation(origin = {-3, 12}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-44, 42}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = if UEL_ex <> 1 then -Modelica.Constants.inf else 0;
equation
  connect(dGREF, gast1.dGREF) annotation(Line(points = {{-60, -20}, {-36.0548, -20}, {-36.0548, -9.0137}, {-20.2808, -9.0137}, {-20.2808, -9.0137}}));
  connect(dVREF, esst1a1.dVREF) annotation(Line(points = {{-60, -40}, {-39.3563, -40}, {-39.3563, 3.26157}, {-18.2648, 3.26157}, {-18.2648, 3.26157}}));
  connect(genrou1.SLIP, pss2b1.PSS_AUX2[1]) annotation(Line(points = {{54, -12}, {56.0991, -12}, {56.0991, -50.2282}, {-87.8451, -50.2282}, {-87.8451, -1.08719}, {-75.2336, -1.08719}, {-75.2336, -1.08719}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, pss2b1.PSS_AUX2[2]) annotation(Line(points = {{54, -3.9}, {59.3606, -3.9}, {59.3606, -56.3165}, {-87.8451, -56.3165}, {-87.8451, -1.08719}, {-75.6685, -1.08719}, {-75.6685, -1.08719}}, color = {0, 0, 127}));
  connect(genrou1.VI, pss2b1.VI2) annotation(Line(points = {{54, 0}, {60.9604, 0}, {60.9604, -59.019}, {-80.3745, -59.019}, {-80.3745, 1.94141}, {-75.3268, 1.94141}, {-75.3268, 1.94141}}, color = {0, 0, 127}));
  connect(genrou1.VI, esst1a1.VI) annotation(Line(points = {{54, 0}, {61.1545, 0}, {61.1545, 20.7731}, {13.3957, 20.7731}, {13.3957, 20.7731}}, color = {0, 0, 127}));
  connect(pss2b1.VI2, pss2b1.VI) annotation(Line(points = {{-75, 2}, {-80.3745, 2}, {-80.3745, 8.15393}, {-75.521, 8.15393}, {-75.521, 8.15393}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, gast1.MBASE) annotation(Line(points = {{54, -8.1}, {59.1091, -8.1}, {59.1091, -56.3356}, {-28.6012, -56.3356}, {-28.6012, -27.5611}, {-20.6275, -27.5611}, {-20.6275, -27.5611}}, color = {0, 0, 127}));
  connect(genrou1.VI, gast1.VI) annotation(Line(points = {{54, 0}, {61.0158, 0}, {61.0158, -58.9358}, {-29.8146, -58.9358}, {-29.8146, -20.6275}, {-19.7608, -20.6275}, {-19.7608, -20.6275}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{39, 60}, {39.1102, 60}, {39.1102, 15}, {39, 15}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{39, -15}, {40, -15}, {40, -38}, {84, -38}, {84, -40}}));
  connect(esst1a1.EFD0, genrou1.EFD0) annotation(Line(points = {{12, 0}, {23.7, 0}}, color = {0, 0, 127}));
  connect(esst1a1.EFD, genrou1.EFD) annotation(Line(points = {{12, 3}, {18, 3}, {18, 4.2}, {24, 4.2}}, color = {0, 0, 127}));
  connect(esst1a1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{12, 6}, {20, 6}, {20, 7.5}, {23.7, 7.5}}, color = {0, 0, 127}));
  connect(esst1a1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{12, 9.9}, {20, 9.9}, {20, 12}, {23.7, 12}}, color = {0, 0, 127}));
  connect(esst1a1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-18, 19.5}, {-24, 19.5}, {-24, 38}, {62, 38}, {62, 7.5}, {54, 7.5}}, color = {0, 0, 127}));
  connect(gast1.PMECH, genrou1.PMECH) annotation(Line(points = {{10, -9}, {24, -9}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, gast1.PMECH0) annotation(Line(points = {{24, -12}, {12, -12}, {12, -38}, {-26, -38}, {-26, -12}, {10, -12}}, color = {0, 0, 127}));
  connect(const.y, esst1a1.VOEL) annotation(Line(points = {{-36.5, 26}, {-30, 26}, {-30, 10.8}, {-18, 10.8}}, color = {0, 0, 127}));
  connect(constant1.y, esst1a1.VUEL) annotation(Line(points = {{-38.5, 42}, {-26, 42}, {-26, 15}, {-18, 15}}, color = {0, 0, 127}));
  connect(pss2b1.VOTHSG, esst1a1.VOTHSG) annotation(Line(points = {{-45, 0}, {-32, 0}, {-32, 6.6}, {-18, 6.6}}, color = {0, 0, 127}));
  connect(pss2b1.PSS_AUX, pss2b1.PSS_AUX2) annotation(Line(points = {{-75, 6}, {-84, 6}, {-84, 0}, {-74.7, 0}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, gast1.SLIP) annotation(Line(points = {{54, -12}, {56, -12}, {56, -50}, {-28, -50}, {-28, -33}, {-20, -33}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_ESST1A_PSS2B_GAST"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN"), Text(origin = {-68.1086, 61.7315}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "dGREF"), Text(origin = {-73.8448, -60.1801}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "TRIP")}));
end GENROU_ESST1A_PSS2B_GAST;
