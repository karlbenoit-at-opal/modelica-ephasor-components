within OpalRT.GenUnits.GENROU;
class GENROU_EXST1_PSS2A_GAST2A
parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
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
  // GAST2A Parameters
  parameter String TG_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real W_tg = 25 "governor gain (1/droop) (on turbine rating)" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real X_tg = 1e-7 "governor lead time constant, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Y_tg = 0.05 "(> 0) governor lag time constant, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Z_tg = 1 "governor mode: 1 Droop, 0 ISO" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real ETD_tg = 0.04 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TCD_tg = 0.2 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TRATE_tg = 1000 "trubine rating, MW" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T_tg = 0.031 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MAX_tg = 1.1 "Maximum limit on turbine rating, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MIN_tg = -0.13 "Minimum limit on turbine rating, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real ECR_tg = 0.01 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K3_tg = 0.9 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real a_tg = 1 "(> 0) valve positioner" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real b_tg = 1 "(> 0) valve positioner, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real c_tg = 1 "valve positioner" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Tf_tg = 0.5 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Kf_tg = 0 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K5_tg = 0.2 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K4_tg = 0.8 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T3_tg = 15 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T4_tg = 2.5 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Tt_tg = 300 "(> 0)" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T5_tg = 0.031 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real af1_tg = 700 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real bf1_tg = 550 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real af2_tg = -0.64 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real bf2_tg = 1.36 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real cf2_tg = 1 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TR_T_tg = 750 "Rated temperature" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K6_tg = 0.25 "Minimum fuel flow, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TC_T_tg = 900 "Temperature control" annotation(Dialog(tab = "GAST2A Parameters"));
  // PSS2A Parameters
  parameter String PSS_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW1_pss = 10 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW2_pss = 10 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T6_pss = 0 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW3_pss = 10 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW4_pss = 0 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T7_pss = 10 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS2_pss = 1.13 "T7/(2*H)" annotation(Dialog(tab = "PSS2A Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T8_pss = 0.3 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T9_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS1_pss = 20 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T1_pss = 0.16 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T2_pss = 0.02 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T3_pss = 0.16 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T4_pss = 0.02 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMAX_pss = 0.2 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMIN_pss = -0.066 annotation(Dialog(tab = "PSS2A Parameters"));
  // PSS2A ICONs
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M1_pss = 0 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M2_pss = 0 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M3_pss = 0 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M4_pss = 3 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M5_pss = 2 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  OpalRT.Electrical.Control.TurbineGovernor.GAST2A gast2a1(ID = TG_ID, W = W_tg, X = X_tg, Y = Y_tg, Z = Z_tg, ETD = ETD_tg, TCD = TCD_tg, TRATE = TRATE_tg, T = T_tg, MAX = MAX_tg, MIN = MIN_tg, ECR = ECR_tg, K3 = K3_tg, a = a_tg, b = b_tg, c = c_tg, Tf = Tf_tg, Kf = Kf_tg, K5 = K5_tg, K4 = K4_tg, T3 = T3_tg, T4 = T4_tg, Tt = Tt_tg, T5 = T5_tg, af1 = af1_tg, bf1 = bf1_tg, af2 = af2_tg, bf2 = bf2_tg, cf2 = cf2_tg, TR = TR_T_tg, K6 = K6_tg, TC = TC_T_tg) annotation(Placement(visible = true, transformation(origin = {-29, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(ID = EX_ID, TR = TR_ex, VIMIN = VIMIN_ex, VIMAX = VIMAX_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {-28, 9}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {52, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 50}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-75, 29}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-70, -10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-70, -20}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(ID = PSS_ID, KS1 = KS1_pss, KS2 = KS2_pss, KS3 = KS3_pss, M0 = M0_pss, M1 = M1_pss, M2 = M2_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T6 = T6_pss, T7 = T7_pss, T8 = T8_pss, T9 = T9_pss, TW1 = TW1_pss, TW2 = TW2_pss, TW3 = TW3_pss, TW4 = TW4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss)  annotation(Placement(visible = true, transformation(origin = {-60, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(pss2a1.VOTHSG, exst11.VOTHSG) annotation(Line(points = {{-50, 4}, {-46.5233, 4}, {-46.5233, 5.50805}, {-37.8678, 5.50805}, {-37.8678, 5.50805}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{10, -2.6}, {15.9473, -2.6}, {15.9473, -33.9747}, {-75.5764, -33.9747}, {-75.5764, 3.98683}, {-69.3362, 3.98683}, {-69.3362, 3.98683}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{10, -8}, {14.2139, -8}, {14.2139, -33.9747}, {-75.5764, -33.9747}, {-75.5764, 3.81349}, {-70.3762, 3.81349}, {-70.3762, 3.81349}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points = {{-70, 4}, {-75.0564, 4}, {-75.0564, 9.88041}, {-70.0295, 9.88041}, {-70.0295, 9.88041}}, color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points = {{-70, 6}, {-77.1365, 6}, {-77.1365, 11.9605}, {-69.6829, 11.9605}, {-69.6829, 11.9605}}, color = {0, 0, 127}));
  connect(genrou1.VI, pss2a1.VI) annotation(Line(points = {{10, 0}, {16.814, 0}, {16.814, -36.9215}, {-77.1365, -36.9215}, {-77.1365, 11.9605}, {-70.2029, 11.9605}, {-70.2029, 11.9605}}, color = {0, 0, 127}));
  connect(dVREF, gast2a1.dGREF) annotation(Line(points = {{-70, -20}, {-46.3143, -20}, {-46.3143, -6.30571}, {-39.1389, -6.30571}, {-39.1389, -6.30571}}));
  connect(dGREF, exst11.dVREF) annotation(Line(points = {{-70, -10}, {-43.2702, -10}, {-43.2702, 3.26157}, {-37.6168, 3.26157}, {-37.6168, 3.26157}}));
  connect(const.y, exst11.VUEL) annotation(Line(points = {{-69.5, 29}, {-44, 29}, {-44, 11}, {-38, 11}}, color = {0, 0, 127}));
  connect(exst11.VI, genrou1.VI) annotation(Line(points = {{-18, 15}, {16.6407, 15}, {16.6407, 0}, {10.0537, 0}, {10.0537, 0}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, gast2a1.MBASE) annotation(Line(points = {{10, -5.4}, {15.0806, -5.4}, {15.0806, -35.7081}, {-43.1618, -35.7081}, {-43.1618, -18.3741}, {-38.8283, -18.3741}, {-38.8283, -18.3741}}, color = {0, 0, 127}));
  connect(genrou1.VI, gast2a1.VI) annotation(Line(points = {{10, 0}, {16.814, 0}, {16.814, -36.9215}, {-44.7218, -36.9215}, {-44.7218, -14.0406}, {-39.1749, -14.0406}, {-39.1749, -14.0406}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{0, 50}, {-0.22779, 50}, {-0.22779, 10}, {0, 10}}));
  connect(bus0, genrou1.p) annotation(Line(points = {{52, -20}, {0, -20}, {0, -10}}, color = {0, 0, 0}));
  connect(exst11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-18, 7.6}, {-14, 7.6}, {-14, 8}, {-10.2, 8}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-18, 5}, {-10.2, 5}}, color = {0, 0, 127}));
  connect(exst11.EFD, genrou1.EFD) annotation(Line(points = {{-18, 3}, {-14, 3}, {-14, 2.8}, {-10, 2.8}}, color = {0, 0, 127}));
  connect(exst11.EFD0, genrou1.EFD0) annotation(Line(points = {{-18, 1}, {-14.5, 1}, {-14.5, 0}, {-10.2, 0}}, color = {0, 0, 127}));
  connect(gast2a1.PMECH, genrou1.PMECH) annotation(Line(points = {{-19, -6}, {-10, -6}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, gast2a1.PMECH0) annotation(Line(points = {{-10, -8}, {-19, -8}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, gast2a1.SLIP) annotation(Line(points = {{10, -8}, {14, -8}, {14, -34}, {-42, -34}, {-42, -22}, {-39, -22}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-38, 14}, {-42, 14}, {-42, 25}, {13, 25}, {13, 5}, {10, 5}}, color = {0, 0, 127}));
  connect(exst11.VOEL, exst11.VUEL) annotation(Line(points = {{-38, 8.2}, {-44, 8.2}, {-44, 11}, {-38, 11}}, color = {0, 0, 127}));
  annotation(experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-124.32, 34.63}, {134.47, -37.46}}, textString = "GENROU_EXST1_PSS2A_GAST2A"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}));
end GENROU_EXST1_PSS2A_GAST2A;
