within OpalRT.GenUnits.GENROU;
class GENROU_EXST1_IEEEST_GAST2A
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // EXST1 Parameters
  parameter Real TR_ex = 0.02 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMAX_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMIN_ex = -1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TC_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TB_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KA_ex = 100 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TA_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMAX_ex = 9 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMIN_ex = -9 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KC_ex = 0.01 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KF_ex = 0.3 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TF_ex = 0.1 annotation(Dialog(tab = "EXST1 Parameters"));
  // GAST2A Parameters
  parameter Real W_tg = 10 "governor gain (1/droop) (on turbine rating)" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real X_tg = 0.01 "governor lead time constant, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Y_tg = 0.5 "(> 0) governor lag time constant, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Z_tg = 0.1 "governor mode: 1 Droop, 0 ISO" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real ETD_tg = 0.1 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TCD_tg = 0.01 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TRATE_tg = 100 "trubine rating, MW" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T_tg = 0.3 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MAX_tg = 6 "Maximum limit on turbine rating, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MIN_tg = -6 "Minimum limit on turbine rating, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real ECR_tg = 0.1 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K3_tg = 0.1 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real a_tg = 0.1 "(> 0) valve positioner" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real b_tg = 0.5 "(> 0) valve positioner, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real c_tg = 0.1 "valve positioner" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Tf_tg = 0.04 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Kf_tg = 0.04 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K5_tg = 0.02 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K4_tg = 2 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T3_tg = 5 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T4_tg = 0.25 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Tt_tg = 150 "(> 0)" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T5_tg = 0.3 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real af1_tg = 100 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real bf1_tg = 150 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real af2_tg = -0.2 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real bf2_tg = 0.3 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real cf2_tg = 0.1 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TR_T_tg = 250 "Rated temperature" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K6_tg = 0.03 "Minimum fuel flow, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TC_T_tg = 150 "Temperature control" annotation(Dialog(tab = "GAST2A Parameters"));
  // IEEEST Parameters
  parameter Real A1_pss = 2 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 2 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 2 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
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
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(TR = TR_ex, VIMIN = VIMIN_ex, VIMAX = VIMAX_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {-30, 9}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {4, -22}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin = {-63, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-58, 34}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {5, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-197, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-66, -3}, extent = {{-4.5, -4.5}, {4.5, 4.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-66, -13}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST2A gast2a1(W = W_tg, X = X_tg, Y = Y_tg, Z = Z_tg, ETD = ETD_tg, TCD = TCD_tg, TRATE = TRATE_tg, T = T_tg, MAX = MAX_tg, MIN = MIN_tg, ECR = ECR_tg, K3 = K3_tg, a = a_tg, b = b_tg, c = c_tg, Tf = Tf_tg, Kf = Kf_tg, K5 = K5_tg, K4 = K4_tg, T3 = T3_tg, T4 = T4_tg, Tt = Tt_tg, T5 = T5_tg, af1 = af1_tg, bf1 = bf1_tg, af2 = af2_tg, bf2 = bf2_tg, cf2 = cf2_tg, TR = TR_T_tg, K6 = K6_tg, TC = TC_T_tg) annotation(Placement(visible = true, transformation(origin = {-30, -17}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{10, -2.6}, {14.7339, -2.6}, {14.7339, -33.9747}, {-76.0965, -33.9747}, {-76.0965, 5.72023}, {-73.323, 5.72023}, {-73.323, 5.72023}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{10, -8}, {11.9605, -8}, {11.9605, -33.9747}, {-76.0965, -33.9747}, {-76.0965, 5.72023}, {-73.4963, 5.72023}, {-73.4963, 5.72023}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX, ieeest1.PSS_AUX2) annotation(Line(points = {{-73, 12}, {-76.4431, 12}, {-76.4431, 5.72023}, {-73.323, 5.72023}, {-73.323, 5.72023}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-73, 8}, {-78.1765, 8}, {-78.1765, 14.0406}, {-73.6697, 14.0406}, {-73.6697, 14.0406}}, color = {0, 0, 127}));
  connect(ieeest1.VI, genrou1.VI) annotation(Line(points = {{-73, 14}, {-78.1765, 14}, {-78.1765, -37.6149}, {16.1207, -37.6149}, {16.1207, -0.17334}, {10.7471, -0.17334}, {10.7471, -0.17334}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, gast2a1.SLIP) annotation(Line(points = {{10, -8}, {12, -8}, {12, -34}, {-44, -34}, {-40, -25}, {-40, -25}}, color = {0, 0, 127}));
  connect(gast2a1.PMECH, genrou1.PMECH) annotation(Line(points = {{-20, -9}, {-16, -9}, {-16, -6}, {-10, -6}}, color = {0, 0, 127}));
  connect(gast2a1.PMECH0, genrou1.PMECH0) annotation(Line(points = {{-20, -11}, {-13.6196, -11}, {-13.6196, -8.20272}, {-10.0599, -8.20272}, {-10.0599, -8.20272}, {-10.0599, -8.20272}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, gast2a1.MBASE) annotation(Line(points = {{10, -5.4}, {12.8458, -5.4}, {12.8458, -35.5967}, {-44.728, -35.5967}, {-44.728, -21.2033}, {-40.085, -21}, {-40, -21}}, color = {0, 0, 127}));
  connect(genrou1.VI, gast2a1.VI) annotation(Line(points = {{10, 0}, {16.0959, 0}, {16.0959, -37.6087}, {-46.4305, -37.6087}, {-46.4305, -17.0245}, {-39.9302, -17}, {-40, -17}}, color = {0, 0, 127}));
  connect(dGREF, gast2a1.dGREF) annotation(Line(points = {{-66, -13}, {-52.3117, -13}, {-52.3117, -9.13133}, {-40, -9}, {-40, -9}}));
  connect(dVREF, exst11.dVREF) annotation(Line(points = {{-66, -3}, {-47.5139, -3}, {-47.5139, 2.9406}, {-40, 2.9406}, {-40, 3}}));
  connect(exst11.VI, genrou1.VI) annotation(Line(points = {{-20, 15}, {16.2507, 15}, {16.2507, -0.154768}, {10.0599, -0.154768}, {10.0599, -0.154768}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{5, 35}, {0.22779, 35}, {0.22779, 9.5672}, {0, 9.5672}, {0, 10}}));
  connect(exst11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-20, 7.6}, {-15, 7.6}, {-15, 8}, {-10.2, 8}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-20, 5}, {-10.2, 5}}, color = {0, 0, 127}));
  connect(exst11.EFD, genrou1.EFD) annotation(Line(points = {{-20, 3}, {-15, 3}, {-15, 2.8}, {-10, 2.8}}, color = {0, 0, 127}));
  connect(exst11.EFD0, genrou1.EFD0) annotation(Line(points = {{-20, 1}, {-16, 1}, {-16, 0}, {-10.2, 0}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, exst11.VOTHSG) annotation(Line(points = {{-53, 6}, {-47, 6}, {-47, 5.4}, {-40, 5.4}}, color = {0, 0, 127}));
  connect(exst11.VOEL, const.y) annotation(Line(points = {{-40, 8.2}, {-50, 8.2}, {-50, 34}, {-52.5, 34}}, color = {0, 0, 127}));
  connect(exst11.VUEL, const.y) annotation(Line(points = {{-40, 11}, {-50, 11}, {-50, 34}, {-52.5, 34}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-40, 14}, {-44, 14}, {-44, 25}, {15, 25}, {15, 5}, {10, 5}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{4, -22}, {0, -22}, {0, -10}}, color = {0, 0, 0}));
  annotation(experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-124.32, 34.63}, {134.47, -37.46}}, textString = "GENROU_EXST1_IEEEST_GAST2A"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}));
end GENROU_EXST1_IEEEST_GAST2A;
