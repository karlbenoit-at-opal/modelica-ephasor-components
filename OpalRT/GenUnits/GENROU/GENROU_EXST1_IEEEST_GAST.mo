within OpalRT.GenUnits.GENROU;
class GENROU_EXST1_IEEEST_GAST
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
  // EXST1 Parameters
  parameter String EX_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TR_ex = 0.02 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMAX_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMIN_ex = 0 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TC_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TB_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KA_ex = 500 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TA_ex = 0.05 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMAX_ex = 8 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMIN_ex = -3 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KC_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KF_ex = 0.1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TF_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
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
  // IEEEST Parameters
  parameter Real A1_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T1_pss = 0.03 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T2_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T3_pss = 0.02 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T4_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T5_pss = 0.2 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T6_pss = 0.1 "(>0)(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real KS_pss = -5 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMAX_pss = 6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMIN_pss = -6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCU_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCL_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  // IEEEST ICONs
  parameter Real M0_pss = 1 "Stabilizer input code" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  parameter Real M1_pss = 1 "IB, remote bus number" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin = {-70, 14}, extent = {{-15, -15}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {31, -3}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-9, -25}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0) annotation(Placement(visible = true, transformation(origin = {-50, 22}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus1 annotation(Placement(visible = true, transformation(origin = {76, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {76, -48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {32, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(ID = EX_ID, TR = TR_ex, VIMIN = VIMIN_ex, VIMAX = VIMAX_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {-5, 14}, extent = {{-20, -20}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-72, -10}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF1 annotation(Placement(visible = true, transformation(origin = {-72, -28}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.VI, ieeest1.VI2) annotation(Line(points = {{46, -3}, {78.8152, -3}, {78.8152, -63.8464}, {-91.0346, -63.8464}, {-91.0346, 6.41519}, {-84.9249, 6.41519}, {-84.9249, 6.41519}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{46, -6.9}, {62.0135, -6.9}, {62.0135, -55.9038}, {-94.3949, -55.9038}, {-94.3949, 3.36034}, {-84.9249, 3.36034}, {-84.9249, 3.36034}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{46, -15}, {51.9325, -15}, {51.9325, -55.9038}, {-94.0894, -55.9038}, {-94.0894, 3.36034}, {-85.5358, 3.36034}, {-85.5358, 3.36034}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-85, 6.5}, {-91.34, 6.5}, {-91.34, 14.0523}, {-85.8413, 14.0523}, {-85.8413, 14.0523}}, color = {0, 0, 127}));
  connect(exst11.VI, genrou1.VI) annotation(Line(points = {{5, 18}, {69.6506, 18}, {69.6506, -3.36034}, {46.7392, -3.36034}, {46.7392, -3.36034}}, color = {0, 0, 127}));
  connect(dGREF1, gast1.dGREF) annotation(Line(points = {{-72, -28}, {-50.0996, -28}, {-50.0996, -12.8304}, {-24.4388, -12.8304}, {-24.4388, -12.8304}}));
  connect(gast1.MBASE, genrou1.MBASE) annotation(Line(points = {{-24, -31}, {-37.5747, -31}, {-37.5747, -60.486}, {58.3477, -60.486}, {58.3477, -11.3029}, {46.7392, -11.3029}, {46.7392, -11.3029}}, color = {0, 0, 127}));
  connect(gast1.VI, genrou1.VI) annotation(Line(points = {{-24, -25}, {-43.6844, -25}, {-43.6844, -63.8464}, {78.8152, -63.8464}, {78.8152, -3.36034}, {47.3502, -3.36034}, {47.3502, -3.36034}}, color = {0, 0, 127}));
  connect(gast1.PMECH0, genrou1.PMECH0) annotation(Line(points = {{6, -16}, {12.5249, -16}, {12.5249, -14.9688}, {16.1907, -14.9688}, {16.1907, -14.9688}}, color = {0, 0, 127}));
  connect(dVREF, exst11.dVREF) annotation(Line(points = {{-72, -10}, {-54.6818, -10}, {-54.6818, 0.305485}, {-26.5772, 0.305485}, {-26.5772, 0.305485}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{32, 52}, {32.1367, 52}, {32.1367, 34}, {32, 34}, {32, 12}, {31, 12}}));
  connect(bus1, genrou1.p) annotation(Line(points = {{76, -48}, {30.3599, -48}, {30.3599, -18.4738}, {31, -18.4738}, {31, -18}}));
  connect(exst11.EFD0, genrou1.EFD0) annotation(Line(points = {{5, -3}, {15.7, -3}}, color = {0, 0, 127}));
  connect(exst11.EFD, genrou1.EFD) annotation(Line(points = {{5, 0}, {10, 0}, {10, 1.2}, {16, 1.2}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{5, 3}, {10.5, 3}, {10.5, 4.5}, {15.7, 4.5}}, color = {0, 0, 127}));
  connect(exst11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{5, 6.9}, {10.5, 6.9}, {10.5, 9}, {15.7, 9}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-25, 16.5}, {-32, 16.5}, {-32, 30}, {54, 30}, {54, 4.5}, {46, 4.5}}, color = {0, 0, 127}));
  connect(gast1.PMECH, genrou1.PMECH) annotation(Line(points = {{6, -13}, {10, -13}, {10, -12}, {16, -12}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, gast1.SLIP) annotation(Line(points = {{46, -15}, {52, -15}, {52, -56}, {-32, -56}, {-32, -37}, {-24, -37}}, color = {0, 0, 127}));
  connect(const1.y, exst11.VOEL) annotation(Line(points = {{-44.5, 22}, {-42, 22}, {-42, 8}, {-25, 8}, {-25, 7.8}}, color = {0, 0, 127}));
  connect(exst11.VUEL, exst11.VOEL) annotation(Line(points = {{-25, 12}, {-42, 12}, {-42, 8}, {-25, 8}, {-25, 7.8}}, color = {0, 0, 127}));
  connect(exst11.VOTHSG, ieeest1.VOTHSG) annotation(Line(points = {{-25, 3.6}, {-42.5, 3.6}, {-42.5, 4}, {-60, 4}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation(Line(points = {{-84.75, 4}, {-96, 4}, {-96, 11.5}, {-85, 11.5}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_EXST1_IEEEST_GAST"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN"), Text(origin = {-68.1086, 61.7315}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "dGREF"), Text(origin = {-73.8448, -60.1801}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "TRIP")}));
end GENROU_EXST1_IEEEST_GAST;
