within OpalRT.GenUnits.GENROU;
class GENROU_EXST1_IEEEST
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
  parameter String EX_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "EXST1 Parameters"));
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
  //
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
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(ID = EX_ID, TR = TR_ex, VIMIN = VIMIN_ex, VIMAX = VIMAX_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {-25, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {6, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {52, -16}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin = {-63, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-58, 31}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-5, 45}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-61, -5}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
equation
  connect(genrou1.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{16, -1.6}, {18.8941, -1.6}, {18.8941, -18.7208}, {-76.7898, -18.7208}, {-76.7898, 5.54689}, {-72.6296, 5.54689}, {-72.6296, 5.54689}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{16, -7}, {17.8541, -7}, {17.8541, -18.7208}, {-76.7898, -18.7208}, {-76.7898, 5.89358}, {-72.9763, 5.89358}, {-72.9763, 5.89358}}, color = {0, 0, 127}));
  connect(genrou1.VI, ieeest1.VI2) annotation(Line(points = {{16, 1}, {19.6224, 1}, {19.6224, -13.2659}, {-75.7259, -13.2659}, {-75.7259, 7.8766}, {-73.2385, 7.8766}, {-73.2385, 7.8766}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-73, 8}, {-75.7259, 8}, {-75.7259, 13.8186}, {-73.2385, 13.8186}, {-73.2385, 13.8186}}, color = {0, 0, 127}));
  connect(dVREF, exst11.dVREF) annotation(Line(points = {{-61, -5}, {-46.7068, -5}, {-46.7068, 3.86921}, {-35.3756, 3.86921}, {-35.3756, 3.86921}}));
  connect(exst11.VI, genrou1.VI) annotation(Line(points = {{-15, 16}, {19.6224, 16}, {19.6224, 0.829116}, {16.3059, 0.829116}, {16.3059, 0.829116}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{-5, 45}, {6, 45}, {6, 11}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{6, -9}, {6, -16}, {52, -16}}));
  connect(exst11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-15, 8.6}, {-9, 8.6}, {-9, 9}, {-4.2, 9}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-15, 6}, {-4.2, 6}}, color = {0, 0, 127}));
  connect(exst11.EFD, genrou1.EFD) annotation(Line(points = {{-15, 4}, {-10, 4}, {-10, 3.8}, {-4, 3.8}}, color = {0, 0, 127}));
  connect(genrou1.EFD0, exst11.EFD0) annotation(Line(points = {{-4.2, 1}, {-10, 1}, {-10, 2}, {-15, 2}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation(Line(points = {{-4, -7}, {-7, -7}, {-7, -5}, {-4, -5}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, exst11.VOTHSG) annotation(Line(points = {{-53, 6}, {-40, 6}, {-40, 6.4}, {-35, 6.4}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-35, 15}, {-40, 15}, {-40, 26}, {19, 26}, {19, 6}, {16, 6}}, color = {0, 0, 127}));
  connect(exst11.VUEL, const.y) annotation(Line(points = {{-35, 12}, {-45, 12}, {-45, 31}, {-52.5, 31}}, color = {0, 0, 127}));
  connect(exst11.VOEL, const.y) annotation(Line(points = {{-35, 9.2}, {-40, 9.2}, {-40, 9}, {-45, 9}, {-45, 31}, {-52.5, 31}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation(Line(points = {{-72.8, 6}, {-77, 6}, {-77, 12}, {-73, 12}}, color = {0, 0, 127}));
  annotation(experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-124.32, 34.63}, {134.47, -37.46}}, textString = "GENROU_EXST1_IEEEST"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}));
end GENROU_EXST1_IEEEST;
