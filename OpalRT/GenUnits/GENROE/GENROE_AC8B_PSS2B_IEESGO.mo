within OpalRT.GenUnits.GENROE;
class GENROE_AC8B_PSS2B_IEESGO
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROE"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROE"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROE"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROE"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROE"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROE"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROE"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROE"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROE"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROE"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROE"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROE"));
  // AC8B Parameters
  parameter Real TR_ex = 0.02 "regulator input filter time constant (sec)" annotation(Dialog(tab = "AC8B"));
  parameter Real KPR_ex = 10 "(pu) (> 0) voltage regulator proportional gain" annotation(Dialog(tab = "AC8B"));
  parameter Real KIR_ex = 0.01 "(pu) voltage regulator integral gain" annotation(Dialog(tab = "AC8B"));
  parameter Real KDR_ex = 2 "(pu) voltage regulator derivative gain" annotation(Dialog(tab = "AC8B"));
  parameter Real TDR_ex = 2 "voltage regulator derivative channel time constant (sec)" annotation(Dialog(tab = "AC8B"));
  parameter Real VPIDMAX_ex = 1 "PID maximum limit" annotation(Dialog(tab = "AC8B"));
  parameter Real VPIDMIN_ex = -1 "PID minimum limit" annotation(Dialog(tab = "AC8B"));
  parameter Real KA_ex = 10 "(pu) voltage regulator gain" annotation(Dialog(tab = "AC8B"));
  parameter Real TA_ex = 0.02 "(sec) regulator time constant" annotation(Dialog(tab = "AC8B"));
  parameter Real VRMAX_ex = 10 "(pu) Voltage regulator output maximum limit" annotation(Dialog(tab = "AC8B"));
  parameter Real VRMIN_ex = -10 "(pu) Voltage regulator output minimum limit" annotation(Dialog(tab = "AC8B"));
  parameter Real KC_ex = 0.02 "rectifier loading factor proportional to commutating reactance" annotation(Dialog(tab = "AC8B"));
  parameter Real KD_ex = 0.02 "demagnetizing factor, function of AC exciter reactances" annotation(Dialog(tab = "AC8B"));
  parameter Real KE_ex = 1 "exciter constant related fo self-excited field" annotation(Dialog(tab = "AC8B"));
  parameter Real TE_ex = 0.02 "exciter time constant (>0)" annotation(Dialog(tab = "AC8B"));
  parameter Real VFEMAX_ex = 10 "exciter field current limit (> 0)" annotation(Dialog(tab = "AC8B"));
  parameter Real VEMIN_ex = -10 "Minimum exciter voltage output" annotation(Dialog(tab = "AC8B"));
  parameter Real E1_ex = 4 "Exciter voltages at which exciter saturation is defined" annotation(Dialog(tab = "AC8B"));
  parameter Real SE_E1_ex = 0.4 "Exciter saturation function value at E1" annotation(Dialog(tab = "AC8B"));
  parameter Real E2_ex = 5 "Exciter voltages at which exciter saturation is defined" annotation(Dialog(tab = "AC8B"));
  parameter Real SE_E2_ex = 0.5 "Exciter saturation function value at E2" annotation(Dialog(tab = "AC8B"));
  // PSS2B Parameters
  parameter Real TW1_pss = 2 ">0" annotation(Dialog(tab = "PSS2B"));
  parameter Real TW2_pss = 2 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2B"));
  parameter Real T6_pss = 0.05 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2B"));
  parameter Real TW3_pss = 2 ">0" annotation(Dialog(tab = "PSS2B"));
  parameter Real TW4_pss = 1.5 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2B"));
  parameter Real T7_pss = 2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2B"));
  parameter Real KS2_pss = 0.259 "T7/(2*H)" annotation(Dialog(tab = "PSS2B"));
  //T7/(2*H);
  parameter Real KS3_pss = 1 annotation(Dialog(tab = "PSS2B"));
  parameter Real T8_pss = 0.5 annotation(Dialog(tab = "PSS2B"));
  parameter Real T9_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2B"));
  parameter Real KS1_pss = 15 annotation(Dialog(tab = "PSS2B"));
  parameter Real T1_pss = 0.15 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2B"));
  parameter Real T2_pss = 0.05 annotation(Dialog(tab = "PSS2B"));
  parameter Real T3_pss = 0.15 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2B"));
  parameter Real T4_pss = 0.05 annotation(Dialog(tab = "PSS2B"));
  parameter Real VSTMAX_pss = 0.1 annotation(Dialog(tab = "PSS2B"));
  parameter Real VSTMIN_pss = -0.1 annotation(Dialog(tab = "PSS2B"));
  parameter Real VS1MAX_pss = 0.1 annotation(Dialog(tab = "PSS2B"));
  parameter Real VS1MIN_pss = -0.1 annotation(Dialog(tab = "PSS2B"));
  parameter Real VS2MAX_pss = 0.1 annotation(Dialog(tab = "PSS2B"));
  parameter Real VS2MIN_pss = -0.1 annotation(Dialog(tab = "PSS2B"));
  parameter Real T10_pss = 0.3 annotation(Dialog(tab = "PSS2B"));
  parameter Real T11_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2B"));
  /// PSS2B ICONs
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2B", group = "ICONs"));
  parameter Real M1_pss = 0 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2B", group = "ICONs"));
  parameter Real M2_pss = 3 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2B", group = "ICONs"));
  parameter Real M3_pss = 0 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2B", group = "ICONs"));
  parameter Real M4_pss = 5 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2B", group = "ICONs"));
  parameter Real M5_pss = 1 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2B", group = "ICONs"));
  // IEESGO parameters
  parameter Real T1_tg = 1 "Controller Lag" annotation(Dialog(tab = "IEESGO"));
  parameter Real T2_tg = 1 "Controller Lead Compensation" annotation(Dialog(tab = "IEESGO"));
  parameter Real T3_tg = 1 "Governor Lag (> 0)" annotation(Dialog(tab = "IEESGO"));
  parameter Real T4_tg = 1 "Delay Due To Steam Inlet Volumes" annotation(Dialog(tab = "IEESGO"));
  parameter Real T5_tg = 1 "Reheater Delay" annotation(Dialog(tab = "IEESGO"));
  parameter Real T6_tg = 1 "Turbine pipe hood Delay" annotation(Dialog(tab = "IEESGO"));
  parameter Real K1_tg = 0.5 "1/Per Unit Regulation" annotation(Dialog(tab = "IEESGO"));
  parameter Real K2_tg = 0.5 "Fraction" annotation(Dialog(tab = "IEESGO"));
  parameter Real K3_tg = 0.5 "fraction" annotation(Dialog(tab = "IEESGO"));
  parameter Real PMAX_tg = 1 "Upper Power Limit" annotation(Dialog(tab = "IEESGO"));
  parameter Real PMIN_tg = -1 "Lower Power Limit" annotation(Dialog(tab = "IEESGO"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROE genroe1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {57, -3}, extent = {{-27, -27}, {27, 27}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-68, 66}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2B pss2b1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID, T10 = T10_pss, T11 = T11_pss, VS1MAX = VS1MAX_pss, VS1MIN = VS1MIN_pss, VS2MAX = VS2MAX_pss, VS2MIN = VS2MIN_pss) annotation(Placement(visible = true, transformation(origin = {-76, 18}, extent = {{-19.5, -13}, {19.5, 13}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEESGO ieesgo1(T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, T4 = T4_tg, T5 = T5_tg, T6 = T6_tg, K1 = K1_tg, K2 = K2_tg, K3 = K3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg) annotation(Placement(visible = true, transformation(origin = {-8, -38}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.AC8B ac8b1(TR = TR_ex, KPR = KPR_ex, KIR = KIR_ex, KDR = KDR_ex, TDR = TDR_ex, VPIDMAX = VPIDMAX_ex, VPIDMIN = VPIDMIN_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, TE = TE_ex, VFEMAX = VFEMAX_ex, VEMIN = VEMIN_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-12, 22}, extent = {{-28.5, -28.5}, {28.5, 28.5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -60}, extent = {{-10.5, -10.5}, {10.5, 10.5}}, rotation = 0), iconTransformation(origin = {103, -57}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-92, -26}, extent = {{-8.5, -8.5}, {8.5, 8.5}}, rotation = 0), iconTransformation(origin = {94, 66}, extent = {{12.25, -12.25}, {-12.25, 12.25}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {58, 58}, extent = {{-10.5, -10.5}, {10.5, 10.5}}, rotation = 0), iconTransformation(origin = {-93.5, 65.5}, extent = {{-11.5, -11.5}, {11.5, 11.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-92, -4}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-94, -60}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
equation
  connect(genroe1.AccPower, pss2b1.PSS_AUX2[2]) annotation(Line(points={{84,
          -10.02},{95.6168,-10.02},{95.6168,-83.7029},{-101.727,-83.7029},{
          -101.727,10.081},{-95.5,10.081},{-95.5,10.85}},                                                                                                                                                          color = {0, 0, 127}));
  connect(genroe1.SLIP, pss2b1.PSS_AUX2[1]) annotation(Line(points={{84,-24.6},
          {90.7291,-24.6},{90.7291,-75.1493},{-101.727,-75.1493},{-101.727,
          10.081},{-95.5,10.081},{-95.5,9.55}},                                                                                                                                                              color = {0, 0, 127}));
  connect(genroe1.VI, pss2b1.VI) annotation(Line(points={{84,-3},{98.3662,-3},{
          98.3662,-89.2016},{-106.309,-89.2016},{-106.309,20.773},{-95.5,20.773},
          {-95.5,20.6}},                                                                                                                                                                    color = {0, 0, 127}));
  connect(pss2b1.PSS_AUX2, pss2b1.PSS_AUX) annotation(Line(points={{-95.5,10.2},
          {-100.199,10.2},{-100.199,17.4126},{-95.5,17.4126},{-95.5,18}},                                                                                             color = {0, 0, 127}));
  connect(pss2b1.VI2, pss2b1.VI) annotation(Line(points={{-95.5,12.8},{-101.421,
          12.8},{-101.421,20.4675},{-95.5,20.4675},{-95.5,20.6}},                                                                                           color = {0, 0, 127}));
  connect(ac8b1.VI, genroe1.VI) annotation(Line(points={{16.5,39.1},{99.1416,
          39.1},{99.1416,-3.00429},{84,-3.00429},{84,-3}},                                                                                               color = {0, 0, 127}));
  connect(genroe1.VI, ieesgo1.VI) annotation(Line(points={{84,-3},{98.2833,-3},
          {98.2833,-89.2704},{-46.7811,-89.2704},{-46.7811,-39.485},{-33,
          -39.485},{-33,-38}},                                                                                                                                                                  color = {0, 0, 127}));
  connect(genroe1.MBASE, ieesgo1.MBASE) annotation(Line(points={{84,-17.58},{
          92.7039,-17.58},{92.7039,-78.97},{-44.6352,-78.97},{-44.6352,-48.4979},
          {-33,-48.4979},{-33,-48}},                                                                                                                                                                         color = {0, 0, 127}));
  connect(dGREF, ieesgo1.dGREF) annotation(Line(points={{-92,-26},{-45.9227,-26},
          {-45.9227,-17.5966},{-33,-17.5966},{-33,-18}}));
  connect(dVREF, ac8b1.dVREF) annotation(Line(points={{-92,-4},{-48.4979,-4},{
          -48.4979,4.29185},{-40.5,4.29185},{-40.5,4.9}}));
  connect(ac8b1.VOEL, const.y) annotation(Line(points={{-40.5,19.72},{-52.5822,
          19.72},{-52.5822,66.1972},{-62.5,66.1972},{-62.5,66}},                                                                                            color = {0, 0, 127}));
  connect(genroe1.EX_AUX, ac8b1.EX_AUX) annotation(Line(points = {{29.46, 18.6}, {18.3099, 18.6}, {16.5, 17.8404}, {16.5, 18.01}}, color = {0, 0, 127}));
  connect(genroe1.ETERM0, ac8b1.ETERM0) annotation(Line(points = {{29.46, 10.5}, {18.7793, 10.5}, {16.5, 9.85915}, {16.5, 10.6}}, color = {0, 0, 127}));
  connect(ac8b1.EFD, genroe1.EFD) annotation(Line(points={{16.5,4.9},{29.5775,
          4.9},{29.5775,4.56},{30,4.56}},                                                                                         color = {0, 0, 127}));
  connect(genroe1.EFD0, ac8b1.EFD0) annotation(Line(points = {{29.46, -3}, {24.8826, -3}, {24.8826, -1.87793}, {17.3709, -0.8}, {16.5, -0.8}}, color = {0, 0, 127}));
  connect(genroe1.XADIFD, ac8b1.XADIFD) annotation(Line(points = {{84, 10.5}, {92.9577, 10.5}, {92.9577, 90.1408}, {-50.2347, 90.1408}, {-50.2347, 35.6808}, {-40.3756, 36.25}, {-40.5, 36.25}}, color = {0, 0, 127}));
  connect(pss2b1.VOTHSG, ac8b1.VOTHSG) annotation(Line(points = {{-56.5, 10.2}, {-47.8873, 10.2}, {-47.8873, 11.7371}, {-41.3146, 11.74}, {-40.5, 11.74}}, color = {0, 0, 127}));
  connect(const.y, ac8b1.VUEL) annotation(Line(points = {{-62.5, 66}, {-52.5822, 66}, {-52.5822, 27.23}, {-41.3146, 27.7}, {-40.5, 27.7}}, color = {0, 0, 127}));
  connect(genroe1.SLIP, ieesgo1.SLIP) annotation(Line(points={{84,-24.6},{
          90.6103,-24.6},{90.6103,-75.1174},{-40.3756,-75.1174},{-40.3756,
          -58.216},{-33,-58.216},{-33,-58}},                                                                                                                                                              color = {0, 0, 127}));
  connect(genroe1.PMECH0, ieesgo1.PMECH0) annotation(Line(points={{30,-24.6},{
          18.3099,-24.6},{18.3099,-23},{17,-23}},                                                                                             color = {0, 0, 127}));
  connect(ieesgo1.PMECH, genroe1.PMECH) annotation(Line(points={{17,-18},{
          28.6385,-18},{28.6385,-19.2},{30,-19.2}},                                                                                     color = {0, 0, 127}));
  connect(genroe1.p, bus0) annotation(Line(points={{57,-30},{57.7465,-30},{
          57.7465,-60},{60,-60}}));
  connect(TRIP, genroe1.TRIP) annotation(Line(points={{58,58},{57.277,58},{
          57.277,24},{57,24}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-53.2285, 16.2102}, extent = {{-38.61, 12.3}, {149.418, -36.3081}}, textString = "GENROE_AC8B_PSS2B_IEESGO"), Rectangle(origin = {1.31663, -1.56948}, extent = {{-99.5444, 98.7472}, {97.197, -96.3998}}), Text(origin = {83.7928, -62.3579}, extent = {{-38.2552, 17.2112}, {4.56, -5.59}}, textString = "bus0"), Text(origin = {62.6776, 65.9597}, extent = {{-24.18, 12.91}, {18.5462, -8.21516}}, textString = "dGREF"), Text(origin = {-63.6192, 65.7259}, extent = {{-18.08, 14.08}, {18.08, -14.08}}, textString = "TRIP"), Text(origin = {-58.6874, -55.3946}, extent = {{-19.25, 14.55}, {28.1702, -21.1228}}, textString = "dVREF")}));
end GENROE_AC8B_PSS2B_IEESGO;
