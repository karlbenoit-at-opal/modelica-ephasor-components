within OpalRT.GenUnits.GENROU;
model GENROU_EXST1_PSS2A_TGOV1
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
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(ID = EX_ID, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {2, 11}, extent = {{-17.5, -17.5}, {17.5, 17.5}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.TGOV1 tgov11(R = R_tg, T1 = T1_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, T2 = T2_tg, T3 = T3_tg, Dt = Dt_tg) annotation(Placement(visible = true, transformation(origin = {1, -26}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF1 annotation(Placement(visible = true, transformation(origin = {-67, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-78, 30}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {53, -3}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-37, 28}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {93, -23}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF1 annotation(Placement(visible = true, transformation(origin = {-69, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {51, 41}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points={{73,-8.2},
          {79.0988,-8.2},{79.0988,-46.3683},{-97.9189,-46.3683},{-97.9189,
          23.7296},{-93,23.7296},{-93,24.5}},                                                                                                                                                                     color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points={{73,-19},{
          79.0988,-19},{79.0988,-46.3683},{-97.9189,-46.3683},{-97.9189,23.7296},
          {-93,23.7296},{-93,23.5}},                                                                                                                                                                        color = {0, 0, 127}));
  connect(genrou1.VI, pss2a1.VI2) annotation(Line(points={{73,-3},{85.9177,-3},
          {85.9177,-52.3689},{-99.2827,-52.3689},{-99.2827,25.9117},{-93,
          25.9117},{-93,26}},                                                                                                                                                                   color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points={{-93,26},{-99.0099,26},
          {-99.0099,31.9123},{-93,31.9123},{-93,32}},                                                                                                 color = {0, 0, 127}));
  connect(dGREF1, tgov11.dGREF) annotation(Line(points={{-67,-24},{-45.0644,-24},
          {-45.0644,-14.1631},{-14,-14.1631},{-14,-14}}));
  connect(dVREF1, exst11.dVREF) annotation(Line(points={{-69,5},{-53.6481,5},{
          -53.6481,0.429185},{-15.5,0.429185},{-15.5,0.5}}));
  connect(exst11.VI, genrou1.VI) annotation(Line(points={{19.5,21.5},{85.4077,
          21.5},{85.4077,-3.43348},{73,-3.43348},{73,-3}},                                                                                                color = {0, 0, 127}));
  connect(tgov11.VI, genrou1.VI) annotation(Line(points={{-14,-26},{-24.4635,
          -26},{-24.4635,-52.3605},{85.8369,-52.3605},{85.8369,-3.43348},{73,
          -3.43348},{73,-3}},                                                                                                                                                                      color = {0, 0, 127}));
  connect(tgov11.MBASE, genrou1.MBASE) annotation(Line(points={{-14,-32},{
          -21.4592,-32},{-21.4592,-48.927},{82.4034,-48.927},{82.4034,-14.1631},
          {73,-14.1631},{73,-13.8}},                                                                                                                                                                   color = {0, 0, 127}));
  connect(tgov11.SLIP, genrou1.SLIP) annotation(Line(points={{-14,-38},{
          -19.3133,-38},{-19.3133,-46.3519},{78.97,-46.3519},{78.97,-18.8841},{
          73,-18.8841},{73,-19}},                                                                                                                                                                  color = {0, 0, 127}));
  connect(tgov11.PMECH0, genrou1.PMECH0) annotation(Line(points={{16,-17},{
          24.4635,-17},{24.4635,-19.3133},{33,-19.3133},{33,-19}},                                                                                            color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{53, -23}, {93, -23}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{51, 41}, {51.1597, 41}, {51.1597, 17}, {53, 17}}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points={{-93,24},{
          -98,24},{-98,30},{-93,30}},                                                                               color = {0, 0, 127}));
  connect(tgov11.PMECH, genrou1.PMECH) annotation(Line(points = {{16, -14}, {25, -14}, {25, -15}, {33, -15}}, color = {0, 0, 127}));
  connect(exst11.EFD0, genrou1.EFD0) annotation(Line(points = {{19.5, -3}, {32.6, -3}}, color = {0, 0, 127}));
  connect(exst11.EFD, genrou1.EFD) annotation(Line(points = {{19.5, 0.5}, {25.75, 0.5}, {25.75, 2.6}, {33, 2.6}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{19.5, 4}, {26, 4}, {26, 7}, {32.6, 7}}, color = {0, 0, 127}));
  connect(exst11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{19.5, 8.55}, {25.75, 8.55}, {25.75, 13}, {32.6, 13}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-15.5, 19.75}, {-23, 19.75}, {-23, 35}, {79, 35}, {79, 7}, {73, 7}}, color = {0, 0, 127}));
  connect(const.y, exst11.VUEL) annotation(Line(points = {{-31.5, 28}, {-29, 28}, {-29, 14.5}, {-15.5, 14.5}}, color = {0, 0, 127}));
  connect(exst11.VOEL, exst11.VUEL) annotation(Line(points = {{-15.5, 9.6}, {-29, 9.6}, {-29, 14.5}, {-15.5, 14.5}}, color = {0, 0, 127}));
  connect(exst11.VOTHSG, pss2a1.VOTHSG) annotation(Line(points = {{-15.5, 4.7}, {-38, 4.7}, {-38, 19}, {-55, 19}, {-55, 24}, {-63, 24}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Text(origin = {-24.01, 2.54}, extent = {{-63.44, 25.74}, {113.33, -34.4}}, textString = "GENROU_EXST1_PSS2A_TGOV1"), Rectangle(origin = {0.797267, -6.94761}, extent = {{-100.243, 106.593}, {98.5883, -92.8811}}), Text(origin = {-147.107, 75.6569}, extent = {{62.5647, -2.8652}, {113.33, -34.4}}, textString = "TRIP"), Text(origin = {-146.326, -65.403}, extent = {{59.0139, 0.676099}, {115.458, -29.1991}}, textString = "dVREF"), Text(origin = {-25.9223, -58.4739}, extent = {{74.6167, -9.25298}, {113.33, -34.4}}, textString = "PIN"), Text(origin = {-145.78, -24.67}, extent = {{59.01, 0.68}, {115.46, -29.2}}, textString = "dGREF")}));
end GENROU_EXST1_PSS2A_TGOV1;
