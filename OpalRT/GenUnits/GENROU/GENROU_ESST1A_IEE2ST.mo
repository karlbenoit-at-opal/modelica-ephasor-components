within OpalRT.GenUnits.GENROU;
class GENROU_ESST1A_IEE2ST
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
  // IEE2ST Parameters
  parameter Real K1_pss = 1 annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real K2_pss = 1 annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T1_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T2_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T3_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T4_pss = 0.1 "(>0) (sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T5_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T6_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T7_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T8_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T9_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real T10_pss = 0.1 "(sec)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real LSMAX_pss = 1 annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real LSMIN_pss = -1 annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real VCU_pss = 1 "(pu)(if equal zero, ignored.)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real VCL_pss = -1 "(pu)(if equal zero, ignored.)" annotation(Dialog(tab = "IEE2ST Parameters"));
  // ICONs
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "IEE2ST Parameters", group = "ICONs"));
  parameter Real M1_pss = 2 "IB1, first remote bus number. CURRENLY DISABLED" annotation(Dialog(tab = "IEE2ST Parameters", group = "ICONs"));
  parameter Real M2_pss = 3 "ICS2, second stabilizer input code" annotation(Dialog(tab = "IEE2ST Parameters", group = "ICONs"));
  parameter Real M3_pss = 0 "B2, second remote bus number CURRENLY DISABLED" annotation(Dialog(tab = "IEE2ST Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.Stabilizer.IEE2ST iee2st1(K1 = K1_pss, K2 = K2_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, T7 = T7_pss, T8 = T8_pss, T9 = T9_pss, T10 = T10_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss, M2 = M2_pss, M3 = M3_pss) annotation(Placement(visible = true, transformation(origin = {-40, -8}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {55, -20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-18, -26}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {55, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST1A esst1a1(UEL = UEL_ex, VOS = VOS_ex, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, TC1 = TC1_ex, TB1 = TB1_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex, KLR = KLR_ex, ILR = ILR_ex) annotation(Placement(visible = true, transformation(origin = {15, -8}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-18, 6}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-40, -30}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = if UEL_ex <> 1 then -Modelica.Constants.inf else 0;
equation
  connect(dVREF, esst1a1.dVREF) annotation(Line(points = {{-40, -30}, {-3.81349, -30}, {-3.81349, -17.1607}, {-0.520021, -17.1607}, {-0.520021, -17.1607}}));
  connect(genrou1.AccPower, iee2st1.PSS_AUX2[2]) annotation(Line(points = {{70, -23.9}, {75.7498, -23.9}, {75.7498, -47.842}, {-58.069, -47.842}, {-58.069, -14.0406}, {-55.1223, -14.0406}, {-55.1223, -14.0406}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, iee2st1.PSS_AUX2[1]) annotation(Line(points = {{70, -32}, {75.7498, -32}, {75.7498, -47.842}, {-58.069, -47.842}, {-58.069, -14.0406}, {-55.8156, -14.0406}, {-55.8156, -14.0406}}, color = {0, 0, 127}));
  connect(genrou1.VI, iee2st1.VI2) annotation(Line(points = {{70, -20}, {78.4331, -20}, {78.4331, -38.6341}, {-60.3779, -38.6341}, {-60.3779, -12.0368}, {-55.3303, -12.0368}, {-55.3303, -12.0368}}, color = {0, 0, 127}));
  connect(iee2st1.VI2, iee2st1.VI) annotation(Line(points = {{-55, -12}, {-60.1838, -12}, {-60.1838, -5.82424}, {-55.3303, -5.82424}, {-55.3303, -5.82424}}, color = {0, 0, 127}));
  connect(esst1a1.VI, genrou1.VI) annotation(Line(points = {{30, 1}, {78.0032, 1}, {78.0032, -19.9651}, {70.4196, -19.9651}, {70.4196, -19.9651}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation(Line(points = {{40, -32}, {37.9974, -32}, {37.9974, -29}, {40, -29}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{55, 40}, {55.1102, 40}, {55.1102, -5}, {55, -5}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{55, -35}, {56, -35}, {56, -59.9198}, {100, -59.9198}, {100, -60}}));
  connect(esst1a1.EFD0, genrou1.EFD0) annotation(Line(points = {{30, -20}, {39.7, -20}}, color = {0, 0, 127}));
  connect(esst1a1.EFD, genrou1.EFD) annotation(Line(points = {{30, -17}, {36, -17}, {36, -15.8}, {40, -15.8}}, color = {0, 0, 127}));
  connect(esst1a1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{30, -14}, {36, -14}, {36, -12.5}, {39.7, -12.5}}, color = {0, 0, 127}));
  connect(esst1a1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{30, -10.1}, {36, -10.1}, {36, -8}, {39.7, -8}}, color = {0, 0, 127}));
  connect(esst1a1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{0, -0.5}, {-10, -0.5}, {-10, 16}, {78, 16}, {78, -12.5}, {70, -12.5}}, color = {0, 0, 127}));
  connect(iee2st1.VOTHSG, esst1a1.VOTHSG) annotation(Line(points = {{-25, -14}, {-12, -14}, {-12, -13.4}, {0, -13.4}}, color = {0, 0, 127}));
  connect(iee2st1.PSS_AUX2, iee2st1.PSS_AUX) annotation(Line(points = {{-54.7, -14}, {-58, -14}, {-58, -8}, {-55, -8}}, color = {0, 0, 127}));
  connect(constant1.y, esst1a1.VUEL) annotation(Line(points = {{-12.5, 6}, {-12, 6}, {-12, -5}, {0, -5}}, color = {0, 0, 127}));
  connect(const.y, esst1a1.VOEL) annotation(Line(points = {{-12.5, -26}, {-6, -26}, {-6, -9.2}, {0, -9.2}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_ESST1A_IEE2ST"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN"), Text(origin = {-73.8448, -60.1801}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "TRIP")}));
end GENROU_ESST1A_IEE2ST;
