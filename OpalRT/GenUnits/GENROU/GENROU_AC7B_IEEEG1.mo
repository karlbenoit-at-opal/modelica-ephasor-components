within OpalRT.GenUnits.GENROU;
model GENROU_AC7B_IEEEG1
  parameter Real partType = 1;
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
  // AC7B parameters
  parameter Real TR_ex = 0.04 "(sec) regulator input filter time constant" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KPR_ex = 4.24 "(pu) regulator proportional gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KIR_ex = 4.24 "(pu) regulator integral gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KDR_ex = 0 "(pu) regulator derivative gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real TDR_ex = 0 "(sec) regulator derivative block time constant" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VRMAX_ex = 5.79 "(pu) regulator output maximum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VRMIN_ex = -5.79 "(pu) regulator output minimum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KPA_ex = 65.36 "(pu) voltage regulator proportional gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KIA_ex = 59.69 "(pu) voltage regulator integral gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VAMAX_ex = 1 "(pu) regulator output maximum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VAMIN_ex = -0.95 "(pu) regulator output minimum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KP_ex = 4.96 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KL_ex = 10 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KF1_ex = 0.212 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KF2_ex = 0 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KF3_ex = 0 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real TF3_ex = 1 "(sec) time constant (> 0)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KC_ex = 0.18 "(pu) rectifier loading factor proportional to commutating reactance" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KD_ex = 0.02 "(pu) demagnetizing factor, function of AC exciter reactances" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KE_ex = 1 "(pu) exciter constant related fo self-excited field" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real TE_ex = 1.1 "(pu) exciter time constant" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VFEMAX_ex = 6.9 "(pu) exciter field current limit (> 0)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VEMIN_ex = 0 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real E1_ex = 6.67 annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real S_E1_ex = 1.951 annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real S_E2_ex = 0.156 annotation(Dialog(tab = "AC7B Parameters"));
  // IEEEG1 Parameters
  parameter Real JBUS_tg = 0 "Bus Identifier (NOT USED)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real M_tg = 0 "Machine Identifier (NOT USED)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K_tg = 15 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T1_tg = 1.5 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real T2_tg = 0.3 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real T3_tg = 0.02 "(>0)(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real Uo_tg = 0.1 "(pu/sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real Uc_tg = -0.5 "(<0)(pu/sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real PMAX_tg = 0.5 "(pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real PMIN_tg = 0 "(pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real T4_tg = 0.1 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K1_tg = 0.1 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K2_tg = 0.1 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T5_tg = 0.2 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K3_tg = 0.5 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K4_tg = 0.1 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T6_tg = 0.1 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K5_tg = 0.6 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K6_tg = 0.2 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T7_tg = 0.3 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K7_tg = 0.2 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K8_tg = 0.6 annotation(Dialog(tab = "IEEEG1"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {-12, -44}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-168, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-200, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-200, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-144, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.AC7B ac7b1(TR = TR_ex, KPR = KPR_ex, KIR = KIR_ex, KDR = KDR_ex, TDR = TDR_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KPA = KPA_ex, KIA = KIA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, KP = KP_ex, KL = KL_ex, KF1 = KF1_ex, KF2 = KF2_ex, KF3 = KF3_ex, TF3 = TF3_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, TE = TE_ex, VFEMAX = VFEMAX_ex, VEMIN = VEMIN_ex, E1 = E1_ex, S_E1 = S_E1_ex, E2 = E2_ex, S_E2 = S_E2_ex) annotation(Placement(visible = true, transformation(origin = {-84, 20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG1 ieeeg11(IBUS = 100, ID = M_ID, K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg, K1 = K1_tg, K2 = K2_tg, T5 = T5_tg, K3 = K3_tg, K4 = K4_tg, T6 = T6_tg, K5 = K5_tg, K6 = K6_tg, T7 = T7_tg, K7 = K7_tg, K8 = K8_tg) annotation(Placement(visible = true, transformation(origin = {-76, -24}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-156, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(dVREF, ac7b1.dVREF) annotation(Line(points = {{-168, 10}, {-144.206, 10}, {-144.206, 4.72103}, {-108.155, 4.72103}, {-108.155, 4.72103}}));
  connect(dGREF, ieeeg11.dGREF) annotation(Line(points = {{-156, -26}, {-115.88, -26}, {-115.88, -12.8755}, {-90.9871, -12.8755}, {-90.9871, -12.8755}}));
  connect(ac7b1.VI, genrou1.VI) annotation(Line(points = {{-59, 35}, {21.8884, 35}, {21.8884, -0.429185}, {5.5794, -0.429185}, {5.5794, -0.429185}}, color = {0, 0, 127}));
  connect(genrou1.VI, ieeeg11.VI) annotation(Line(points = {{5, 0}, {21.8884, 0}, {21.8884, -64.3777}, {-111.588, -64.3777}, {-111.588, -24.4635}, {-90.9871, -24.4635}, {-90.9871, -24.4635}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, ieeeg11.MBASE) annotation(Line(points = {{5, -13.5}, {18.4549, -13.5}, {18.4549, -58.3691}, {-108.155, -58.3691}, {-108.155, -30.4721}, {-92.2747, -30.4721}, {-92.2747, -30.4721}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, ieeeg11.PMECH0) annotation(Line(points = {{-45, -20}, {-56.2232, -20}, {-56.2232, -15.0215}, {-60.0858, -15.0215}, {-60.0858, -15.0215}}, color = {0, 0, 127}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{-20, 25}, {-20.0456, 25}, {-20.0456, 60}, {-20, 60}}, color = {0, 0, 127}));
  connect(ac7b1.EFD0, genrou1.EFD0) annotation(Line(points = {{-59, 0}, {-45.5, 0}}, color = {0, 0, 127}));
  connect(ac7b1.EFD, genrou1.EFD) annotation(Line(points = {{-59, 5}, {-52.5, 5}, {-52.5, 7}, {-45, 7}}, color = {0, 0, 127}));
  connect(ac7b1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-59, 10}, {-52, 10}, {-52, 12.5}, {-45.5, 12.5}}, color = {0, 0, 127}));
  connect(ac7b1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-59, 16.5}, {-52.5, 16.5}, {-52.5, 20}, {-45.5, 20}}, color = {0, 0, 127}));
  connect(ieeeg11.PMECH, genrou1.PMECH) annotation(Line(points = {{-61, -12}, {-54, -12}, {-54, -15}, {-45, -15}}, color = {0, 0, 127}));
  connect(ieeeg11.SLIP, genrou1.SLIP) annotation(Line(points = {{-91, -36}, {-104, -36}, {-104, -54}, {14, -54}, {14, -20}, {5, -20}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{-12, -44}, {-20, -44}, {-20, -25}}, color = {0, 0, 0}));
  connect(ac7b1.VOEL, const.y) annotation(Line(points = {{-109, 18}, {-122, 18}, {-122, 34}, {-133, 34}}, color = {0, 0, 127}));
  connect(ac7b1.VUEL, const.y) annotation(Line(points = {{-109, 25}, {-122, 25}, {-122, 34}, {-133, 34}}, color = {0, 0, 127}));
  connect(ac7b1.VOTHSG, const.y) annotation(Line(points = {{-109, 11}, {-122, 11}, {-122, 34}, {-133, 34}}, color = {0, 0, 127}));
  connect(ac7b1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-109, 32.5}, {-116, 32.5}, {-116, 74}, {16, 74}, {16, 12.5}, {5, 12.5}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-77.0165, 1.65977}, extent = {{-72.35, 23.36}, {106.974, -19.9431}}, textString = "GENROU_AC7B_IEEEG1"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN"), Text(origin = {-147.75, 73.49}, extent = {{-39.43, 24.94}, {7.19, -8.82}}, textString = "TRIP"), Text(origin = {-145.52, -85.0977}, extent = {{-39.43, 24.94}, {20.1741, -8.82}}, textString = "dVREF")}));
end GENROU_AC7B_IEEEG1;
