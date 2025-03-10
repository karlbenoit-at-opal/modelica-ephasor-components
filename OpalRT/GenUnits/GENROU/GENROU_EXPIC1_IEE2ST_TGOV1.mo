within OpalRT.GenUnits.GENROU;
model GENROU_EXPIC1_IEE2ST_TGOV1
 parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // EXPIC1 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KA_ex = 0.2 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA1_ex = 12 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VR1_ex = 5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VR2_ex = -5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA2_ex = 10 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA3_ex = 400 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA4_ex = 5 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TF2_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real EFDMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real EFDMIN_ex = -5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KE_ex = 0.2 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KP_ex = 0.4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KI_ex = 0.5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  // TGOV1 Parameters
  parameter Real R_tg = 0.06 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T1_tg = 0.5 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMAX_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMIN_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T2_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T3_tg = 1 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real Dt_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
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
  // ICONs
  parameter Real VCU_pss = 1 "(pu)(if equal zero, ignored.)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real VCL_pss = -1 "(pu)(if equal zero, ignored.)" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real M1_pss = 2 "IB1, first remote bus number. CURRENLY DISABLED" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real M2_pss = 3 "ICS2, second stabilizer input code" annotation(Dialog(tab = "IEE2ST Parameters"));
  parameter Real M3_pss = 0 "B2, second remote bus number CURRENLY DISABLED" annotation(Dialog(tab = "IEE2ST Parameters"));
  OpalRT.Electrical.Control.Stabilizer.IEE2ST iee2st1(K1 = K1_pss, K2 = K2_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, T7 = T7_pss, T8 = T8_pss, T9 = T9_pss, T10 = T10_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss, M2 = M2_pss, M3 = M3_pss) annotation(Placement(visible = true, transformation(origin = {-77, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-76, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {53, 12}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXPIC1 expic11(TR = TR_ex, KA = KA_ex, TA1 = TA1_ex, VR1 = VR1_ex, VR2 = VR2_ex, TA2 = TA2_ex, TA3 = TA3_ex, TA4 = TA4_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, EFDMAX = EFDMAX_ex, EFDMIN = EFDMIN_ex, KE = KE_ex, TE = TE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, KP = KP_ex, KI = KI_ex, KC = KC_ex) annotation(Placement(visible = true, transformation(origin = {-4, 30}, extent = {{-22.5, -22.5}, {22.5, 22.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-52, 43}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {68, -22}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {51, 56}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-94, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.TGOV1 tgov11(R = R_tg, T1 = T1_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, T2 = T2_tg, T3 = T3_tg, Dt = Dt_tg) annotation(Placement(visible = true, transformation(origin = {4, -10}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-76, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.AccPower, iee2st1.PSS_AUX2[2]) annotation(Line(points = {{73, 6.8}, {82.8005, 6.8}, {82.8005, -37.2602}, {-91.3241, -37.2602}, {-91.3241, 43.8356}, {-87.9146, 43.8356}, {-87.9146, 43.8356}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, iee2st1.PSS_AUX2[1]) annotation(Line(points = {{73, -4}, {78.904, -4}, {78.904, -37.2602}, {-91.3241, -37.2602}, {-91.3241, 43.592}, {-86.697, 43.592}, {-86.697, 43.592}}, color = {0, 0, 127}));
  connect(iee2st1.VI2, iee2st1.VI) annotation(Line(points = {{-87, 46}, {-89.6193, 46}, {-89.6193, 52.1156}, {-86.9405, 52.1156}, {-86.9405, 52.1156}}, color = {0, 0, 127}));
  connect(genrou1.VI, iee2st1.VI2) annotation(Line(points = {{73, 12}, {84.0181, 12}, {84.0181, -42.8614}, {-93.5158, -42.8614}, {-93.5158, 46.5144}, {-87.184, 46.5144}, {-87.184, 46.5144}}, color = {0, 0, 127}));
  connect(expic11.VI, genrou1.VI) annotation(Line(points = {{18.5, 43.5}, {84.2617, 43.5}, {84.2617, 11.933}, {74.2769, 11.933}, {74.2769, 11.933}}, color = {0, 0, 127}));
  connect(genrou1.VI, tgov11.VI) annotation(Line(points = {{73, 12}, {84.0181, 12}, {84.0181, -42.8614}, {-24.5966, -42.8614}, {-24.5966, -9.98476}, {-10.9589, -9.98476}, {-10.9589, -9.98476}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, tgov11.MBASE) annotation(Line(points = {{73, 1.2}, {81.3393, 1.2}, {81.3393, -39.9391}, {-22.1613, -39.9391}, {-22.1613, -16.073}, {-10.9589, -16.073}, {-10.9589, -16.073}}, color = {0, 0, 127}));
  connect(tgov11.PMECH0, genrou1.PMECH0) annotation(Line(points = {{19, -1}, {25.3272, -1}, {25.3272, -4.62709}, {33.6073, -4.62709}, {33.6073, -4.62709}}, color = {0, 0, 127}));
  connect(dVREF, expic11.dVREF) annotation(Line(points = {{-76, 22}, {-57.4733, 22}, {-57.4733, 15.8295}, {-27.2755, 15.8295}, {-27.2755, 15.8295}}));
  connect(dGREF, tgov11.dGREF) annotation(Line(points = {{-76, 0}, {-16.8036, 0}, {-16.8036, 1.46119}, {-11.2024, 1.46119}, {-11.2024, 1.46119}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{51, 56}, {51.1597, 56}, {51.1597, 32}, {53, 32}}));
  connect(expic11.EFD0, genrou1.EFD0) annotation(Line(points = {{18.5, 12}, {32.6, 12}}, color = {0, 0, 127}));
  connect(expic11.EFD, genrou1.EFD) annotation(Line(points = {{18.5, 16.5}, {24.75, 16.5}, {24.75, 17.6}, {33, 17.6}}, color = {0, 0, 127}));
  connect(expic11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{18.5, 21}, {25, 21}, {25, 22}, {32.6, 22}}, color = {0, 0, 127}));
  connect(expic11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{18.5, 26.85}, {26.75, 26.85}, {26.75, 28}, {32.6, 28}}, color = {0, 0, 127}));
  connect(tgov11.PMECH, genrou1.PMECH) annotation(Line(points = {{19, 2}, {26, 2}, {26, 0}, {33, 0}}, color = {0, 0, 127}));
  connect(expic11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-26.5, 41.25}, {-36, 41.25}, {-36, 66}, {86, 66}, {86, 22}, {73, 22}}, color = {0, 0, 127}));
  connect(const.y, expic11.VUEL) annotation(Line(points = {{-46.5, 43}, {-44, 43}, {-44, 34.5}, {-26.5, 34.5}}, color = {0, 0, 127}));
  connect(expic11.VOEL, expic11.VUEL) annotation(Line(points = {{-26.5, 28.2}, {-44, 28.2}, {-44, 34.5}, {-26.5, 34.5}}, color = {0, 0, 127}));
  connect(tgov11.SLIP, genrou1.SLIP) annotation(Line(points = {{-11, -22}, {-21, -22}, {-21, -37}, {79, -37}, {79, -4}, {73, -4}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{68, -22}, {53, -22}, {53, -8}}, color = {0, 0, 0}));
  connect(expic11.VOTHSG, iee2st1.VOTHSG) annotation(Line(points = {{-26.5, 21.9}, {-49, 21.9}, {-49, 32}, {-62, 32}, {-62, 44}, {-67, 44}}, color = {0, 0, 127}));
  connect(iee2st1.PSS_AUX2, iee2st1.PSS_AUX) annotation(Line(points = {{-87, 44}, {-91, 44}, {-91, 56}, {-87, 56}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Text(origin = {-24.01, 2.54}, extent = {{-63.44, 25.74}, {113.33, -34.4}}, textString = "GENROU_EXPIC1_IEE2ST_TGOV1"), Rectangle(origin = {0.797267, -6.94761}, extent = {{-100.243, 106.593}, {98.5883, -92.8811}}), Text(origin = {-147.107, 75.6569}, extent = {{62.5647, -2.8652}, {113.33, -34.4}}, textString = "TRIP"), Text(origin = {-146.326, -65.403}, extent = {{59.0139, 0.676099}, {115.458, -29.1991}}, textString = "dVREF"), Text(origin = {-25.9223, -58.4739}, extent = {{74.6167, -9.25298}, {113.33, -34.4}}, textString = "PIN"), Text(origin = {-145.78, -24.67}, extent = {{59.01, 0.68}, {115.46, -29.2}}, textString = "dGREF")}));
end GENROU_EXPIC1_IEE2ST_TGOV1;
