within OpalRT.GenUnits.GENROU;
class GENROU_EXST3_PSS2A
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "General"));
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
  // EXST2 Parameters
  parameter Real TR_ex = 0 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VIMAX_ex = 0.2 "(pu)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VIMIN_ex = -0.2 "(pu)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KJ_ex = 100 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real TC_ex = 1 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real TB_ex = 10 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KA_ex = 200 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real TA_ex = 0 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VRMAX_ex = 10 "or zero" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VRMIN_ex = -10 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KG_ex = 1.0 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KP_ex = 6.15 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KI_ex = 0 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real XL_ex = 0.081 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KC_ex = 0.2 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real EFDMAX_ex = 6.9 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VGMAX_ex = 5.8 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real THETAP_ex = 0 annotation(Dialog(tab = "EXST2 Parameters"));
  //****************************
  // PSS2A Parameters
  parameter Real TW1_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW2_pss = 1 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T6_pss = 1 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW3_pss = 1.5 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW4_pss = 0.2 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T7_pss = 0.2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS2_pss = 0.1 "T7/(2*H)" annotation(Dialog(tab = "PSS2A Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 0.1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T8_pss = 0.05 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T9_pss = 0.01 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS1_pss = 0.15 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T1_pss = 0.1 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T2_pss = 0.1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T3_pss = 0.01 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T4_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMAX_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMIN_pss = -1 annotation(Dialog(tab = "PSS2A Parameters"));
  // PSS2A ICONs
  parameter Real M0_pss = 2 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M1_pss = 2 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M2_pss = 2 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M3_pss = 2 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M4_pss = 1 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M5_pss = 2 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  //
  //
  //****************************
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {-56, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {-26, 32}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-63, 63}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-200, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST3 exst31(TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, KJ = KJ_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KG = KG_ex, KP = KP_ex, KI = KI_ex, EFDMAX = EFDMAX_ex, KC = KC_ex, XL = XL_ex, VGMAX = VGMAX_ex, THETAP = THETAP_ex) annotation(Placement(visible = true, transformation(origin = {-84, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-128, 56}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-140, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-124, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.VI, pss2a1.VI2) annotation(Line(points = {{-46, 42}, {-39.6048, 42}, {-39.6048, 20.9673}, {-147.547, 20.9673}, {-147.547, 52.0299}, {-143.665, 52.0299}, {-143.665, 52.0299}}, color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points = {{-143, 52}, {-147.547, 52}, {-147.547, 58.2424}, {-143.665, 58.2424}, {-143.665, 58.2424}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{-46, 39.4}, {-37.8576, 39.4}, {-37.8576, 18.6376}, {-149.877, 18.6376}, {-149.877, 49.8943}, {-142.5, 49.8943}, {-142.5, 49.8943}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{-46, 34}, {-39.799, 34}, {-39.799, 24.2677}, {-149.877, 24.2677}, {-149.877, 49.8943}, {-143.276, 49.8943}, {-143.276, 49.8943}}, color = {0, 0, 127}));
  connect(exst31.VI, genrou1.VI) annotation(Line(points = {{-74, 56}, {-39.6048, 56}, {-39.6048, 41.9345}, {-45.8173, 41.9345}, {-45.8173, 41.9345}}, color = {0, 0, 127}));
  connect(dVREF, exst31.dVREF) annotation(Line(points = {{-140, 40}, {-106.972, 40}, {-106.972, 44.0701}, {-94.1585, 44.0701}, {-94.1585, 44.0701}}));
  connect(bus0, genrou1.p) annotation(Line(points = {{-26, 32}, {-56, 32}}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{-56, 52}, {-56, 63}, {-63, 63}}, color = {0, 0, 127}));
  connect(exst31.EFD0, genrou1.EFD0) annotation(Line(points = {{-74, 42}, {-66.2, 42}}, color = {0, 0, 127}));
  connect(exst31.EFD, genrou1.EFD) annotation(Line(points = {{-74, 44}, {-70, 44}, {-70, 44.8}, {-66, 44.8}}, color = {0, 0, 127}));
  connect(exst31.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-74, 46}, {-70, 46}, {-70, 47}, {-66.2, 47}}, color = {0, 0, 127}));
  connect(exst31.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-74, 48.6}, {-70, 48.6}, {-70, 50}, {-66.2, 50}}, color = {0, 0, 127}));
  connect(exst31.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-94, 55}, {-98, 55}, {-98, 74}, {-42, 74}, {-42, 47}, {-46, 47}}, color = {0, 0, 127}));
  connect(exst31.VOTHSG, pss2a1.VOTHSG) annotation(Line(points = {{-94, 46.4}, {-102, 46.4}, {-102, 50}, {-113, 50}}, color = {0, 0, 127}));
  connect(exst31.VUEL, const.y) annotation(Line(points = {{-94, 52}, {-100, 52}, {-100, 80}, {-113, 80}}, color = {0, 0, 127}));
  connect(exst31.VOEL, const.y) annotation(Line(points = {{-94, 49.2}, {-100, 49.2}, {-100, 80}, {-113, 80}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points = {{-142.7, 50}, {-150, 50}, {-150, 56}, {-143, 56}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation(Line(points = {{-66, 34}, {-70, 34}, {-70, 36}, {-66, 36}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-72.35, 23.36}, {72.35, -23.36}}, textString = "GENROU_EXST3_PSS2A"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}));
end GENROU_EXST3_PSS2A;
