within OpalRT.GenUnits.GENROU;
class GENROU_EXAC1_PSS2A_TGOV1
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
  // EXAC1 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TF_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXAC1 Parameters"));
  // TGOV1 Parameters
  parameter Real R_tg = 0.06 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T1_tg = 0.5 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMAX_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMIN_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T2_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T3_tg = 1 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real Dt_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  // PSS2A Parameters
  parameter Real TW1_pss = 1 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW2_pss = 4 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T6_pss = 0.01 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW3_pss = 1 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW4_pss = 1 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T7_pss = 2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS2_pss = 0.4 "T7/(2*H)" annotation(Dialog(tab = "PSS2A Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 10 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T8_pss = 0.3 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T9_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS1_pss = 20 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T1_pss = 0.4 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T2_pss = 0.07 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T3_pss = 0.3 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T4_pss = 0.06 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMAX_pss = 0.5 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMIN_pss = -0.5 annotation(Dialog(tab = "PSS2A Parameters"));
  // PSS2A ICONs
  parameter Real M0_pss = 2 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M1_pss = 0 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M2_pss = 2 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M3_pss = 0 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M4_pss = 1 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M5_pss = 1 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.Excitation.EXAC1 exac11(ID = M_ID, TR = TR_ex, TB = TB_ex, TC = TC_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {10, 31}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-59, 36}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {50, 19}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0) annotation(Placement(visible = true, transformation(origin = {-23, 46}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.TGOV1 tgov11(R = R_tg, T1 = T1_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, T2 = T2_tg, T3 = T3_tg, Dt = Dt_tg) annotation(Placement(visible = true, transformation(origin = {9, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {55, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -10}, extent = {{-4.5, -4.5}, {4.5, 4.5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-35, 22}, extent = {{-5.75, -5.75}, {5.75, 5.75}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-23, 12}, extent = {{-5.5, -5.5}, {5.5, 5.5}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{65, 15.1}, {79.0155, 15.1}, {79.0155, -20.1907}, {-79.0155, -20.1907}, {-79.0155, 29.5095}, {-74.3561, 29.5095}, {-74.3561, 29.5095}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{65, 7}, {68.9202, 7}, {68.9202, -19.9966}, {-79.0155, -19.9966}, {-79.0155, 29.7036}, {-74.9385, 29.7036}, {-74.9385, 29.7036}}, color = {0, 0, 127}));
  connect(genrou1.VI, pss2a1.VI2) annotation(Line(points = {{65, 19}, {81.5393, 19}, {81.5393, -28.7329}, {-77.4624, -28.7329}, {-77.4624, 32.0333}, {-74.7444, 32.0333}, {-74.7444, 32.0333}}, color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points = {{-74, 32}, {-77.2682, 32}, {-77.2682, 38.2458}, {-73.9678, 38.2458}, {-73.9678, 38.2458}}, color = {0, 0, 127}));
  connect(tgov11.VI, genrou1.VI) annotation(Line(points = {{-6, 0}, {-31.4509, 0}, {-31.4509, -28.7329}, {81.5393, -28.7329}, {81.5393, 19.0258}, {65.6198, 19.0258}, {65.6198, 19.0258}}, color = {0, 0, 127}));
  connect(dGREF, tgov11.dGREF) annotation(Line(points = {{-23, 12}, {-17.5966, 12}, {-17.5966, 12.4464}, {-6, 12.4464}, {-6, 12}}));
  connect(dVREF, exac11.dVREF) annotation(Line(points = {{-35, 22}, {-24.8927, 22}, {-24.8927, 21.8884}, {-5, 21.8884}, {-5, 22}}));
  connect(tgov11.MBASE, genrou1.MBASE) annotation(Line(points = {{-6, -6}, {-31.0626, -6}, {-31.0626, -24.8501}, {76.6858, -24.8501}, {76.6858, 10.6778}, {65.4256, 10.6778}, {65.4256, 10.6778}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{60, -10}, {49, -10}, {49, 5.5262}, {50, 5.5262}, {50, 4}}));
  connect(exac11.VI, genrou1.VI) annotation(Line(points = {{25, 40}, {81.1159, 40}, {81.1159, 18.8841}, {66.0944, 18.8841}, {66.0944, 18.8841}}, color = {0, 0, 127}));
  connect(tgov11.PMECH0, genrou1.PMECH0) annotation(Line(points = {{24, 9}, {27.897, 9}, {27.897, 7.29614}, {34.7639, 7.29614}, {34.7639, 7.29614}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{55, 74}, {54.4533, 74}, {54.4533, 55.8679}, {49.8975, 55.8679}, {49.8975, 34}, {50, 34}}));
  connect(tgov11.PMECH, genrou1.PMECH) annotation(Line(points = {{24, 12}, {28.5, 12}, {28.5, 10}, {35, 10}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points = {{-73.7, 30}, {-79, 30}, {-79, 36}, {-74, 36}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, tgov11.SLIP) annotation(Line(points = {{65, 7}, {69, 7}, {69, -20}, {-27, -20}, {-27, -12}, {-6, -12}}, color = {0, 0, 127}));
  connect(exac11.EFD0, genrou1.EFD0) annotation(Line(points = {{25, 19}, {34.7, 19}}, color = {0, 0, 127}));
  connect(exac11.EFD, genrou1.EFD) annotation(Line(points = {{25, 22}, {30, 22}, {30, 23.2}, {35, 23.2}}, color = {0, 0, 127}));
  connect(exac11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{25, 25}, {30, 25}, {30, 26.5}, {34.7, 26.5}}, color = {0, 0, 127}));
  connect(exac11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{25, 28.9}, {30.5, 28.9}, {30.5, 31}, {34.7, 31}}, color = {0, 0, 127}));
  connect(exac11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-5, 38.5}, {-10, 38.5}, {-10, 50}, {71, 50}, {71, 26.5}, {65, 26.5}}, color = {0, 0, 127}));
  connect(exac11.VUEL, const1.y) annotation(Line(points = {{-5, 34}, {-14, 34}, {-14, 46}, {-17.5, 46}}, color = {0, 0, 127}));
  connect(exac11.VOEL, const1.y) annotation(Line(points = {{-5, 29.8}, {-14, 29.8}, {-14, 46}, {-17.5, 46}}, color = {0, 0, 127}));
  connect(pss2a1.VOTHSG, exac11.VOTHSG) annotation(Line(points = {{-44, 30}, {-17, 30}, {-17, 25.6}, {-5, 25.6}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Rectangle(origin = {1.13895, -0.455581}, extent = {{-100.911, 100}, {98.6333, -99.3166}}), Text(origin = {60.7061, -75.9657}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-20.8475, 43.1708}, extent = {{-73.46, 18.79}, {115.147, -34.9626}}, textString = "GENROU_EXAC1_PSS2A_TGOV1"), Text(origin = {-68.6979, 75.4952}, extent = {{27.4445, -7.63467}, {-20.16, 17.89}}, textString = "TRIP"), Text(origin = {-63.2555, -6.75674}, extent = {{27.44, -7.63}, {-20.16, 17.89}}, textString = "dGREF"), Text(origin = {-63.9645, -84.2314}, extent = {{27.44, -7.63}, {-20.16, 17.89}}, textString = "dVREF")}));
end GENROU_EXAC1_PSS2A_TGOV1;
