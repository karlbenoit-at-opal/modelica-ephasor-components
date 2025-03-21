within OpalRT.GenUnits.GENROU;
class GENROU_EXAC2_GGOV1
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
  // EXAC2 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VAMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VAMIN_ex = -5 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KB_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KL_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KH_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TF_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VLR_ex = 4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXAC2 Parameters"));
  //
  // GGOV1 parameters
  parameter Real R_tg = 0.5 "Permanent droop (pu)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tpelec_tg = 0.6 "Electrical power transducer time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real maxerr_tg = 0.025 "Maximum value for speed error signal" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real minerr_tg = -0.025 "Minimum value for speed error signal" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kpgov_tg = 6 "Governor proportional gain" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kigov_tg = 0.22 "Governor integral gain" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kdgov_tg = 0 "Governor derivative gain" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tdgov_tg = 1 "Governor derivative controller time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real vmax_tg = 1 "Maximum valve position limit" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real vmin_tg = 0.24 "Minimum valve position limit" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tact_tg = 0.6 "Actuator time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kturb_tg = 1.5 "Turbine gain" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Wfnl_tg = 0.25 "No load fuel flow (pu)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tb_tg = 1 "Turbine lag time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tc_tg = 1.1 "Turbine lead time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Teng_tg = 0 "Transport lag time constant for diesel engine (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tfload_tg = 0.3 "Load Limiter time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kpload_tg = 1.2 "Load limiter proportional gain for PI controller" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kiload_tg = 3.3 "Load limiter integral gain for PI controller" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Ldref_tg = 1 "Load limiter reference value (pu)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Dm_tg = -0.5 "Mechanical damping coefficient (pu)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Ropen_tg = 99 "Maximum valve opening rate (pu/sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Rclose_tg = -99 "Maximum valve closing rate (pu/sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Kimw_tg = 0.01 "Power controller (reset) gain" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Aset_tg = 99 "Acceleration limiter setpoint (pu/sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Ka_tg = 10 "Acceleration limiter gain" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Ta_tg = 1 "Acceleration limiter time constant( > 0) (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Trate_tg = 200 "Turbine rating (MW)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real db_tg = 0 "Speed governor deadband" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tsa_tg = 1 "Temperature detection lead time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Tsb_tg = 1 "Temperature detection lag time constant (sec)" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Rup_tg = 99 "Maximum rate of load limit increase" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Rdown_tg = -99 "Maximum rate of load limit decrease" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Rselect_tg = 1 "ICON(M), Feedback signal for governor droop" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real Flag_tg = 1 "ICON(M+1),Switch for fuel source characteristic" annotation(Dialog(tab = "GGOV1 parameters"));
  parameter Real DELTA_tg = 0.01 "Simulation sample time, sec." annotation(Dialog(tab = "GGOV1 parameters"));
  OpalRT.Electrical.Control.TurbineGovernor.GGOV1 ggov11(R = R_tg, Tpelec = Tpelec_tg, maxerr = maxerr_tg, minerr = minerr_tg, Kpgov = Kpgov_tg, Kigov = Kigov_tg, Kdgov = Kdgov_tg, Tdgov = Tdgov_tg, vmax = vmax_tg, vmin = vmin_tg, Tact = Tact_tg, Kturb = Kturb_tg, Wfnl = Wfnl_tg, Tb = Tb_tg, Tc = Tc_tg, Teng = Teng_tg, Tfload = Tfload_tg, Kpload = Kpload_tg, Kiload = Kiload_tg, Ldref = Ldref_tg, Dm = Dm_tg, Ropen = Ropen_tg, Rclose = Rclose_tg, Kimw = Kimw_tg, Aset = Aset_tg, Ka = Ka_tg, Ta = Ta_tg, Trate = Trate_tg, db = db_tg, Tsa = Tsa_tg, Tsb = Tsb_tg, Rup = Rup_tg, Rdown = Rdown_tg, Rselect = Rselect_tg, Flag = Flag_tg, DELTA = DELTA_tg) annotation(Placement(visible = true, transformation(origin = {-22, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {15, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {26, -38}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXAC2 exac21(ID = M_ID, TR = TR_ex, TB = TB_ex, TC = TC_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, KB = KB_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TE = TE_ex, KL = KL_ex, KH = KH_ex, KF = KF_ex, TF = TF_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, VLR = VLR_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-25, -3}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-60, 6}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-66, -30}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-140, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-80, -8}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(ggov11.PMECH0, genrou1.PMECH0) annotation(Line(points = {{-12, -28}, {-3.68868, -28}, {-3.68868, -27.1798}, {-0.388283, -27.1798}, {-0.388283, -27.1798}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ggov11.SLIP) annotation(Line(points = {{30, -27}, {37.8576, -27}, {37.8576, -59.7955}, {-35.5279, -59.7955}, {-35.5279, -41.9345}, {-32.2275, -41.9345}, {-32.2275, -41.9345}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, ggov11.MBASE) annotation(Line(points = {{30, -23.1}, {40.1872, -23.1}, {40.1872, -62.9018}, {-37.6634, -62.9018}, {-37.6634, -37.8576}, {-32.2275, -37.8576}, {-32.2275, -37.8576}}, color = {0, 0, 127}));
  connect(genrou1.VI, ggov11.VI) annotation(Line(points = {{30, -15}, {43.8759, -15}, {43.8759, -65.8139}, {-39.799, -65.8139}, {-39.799, -34.5572}, {-32.4216, -34.5572}, {-32.4216, -34.5572}}, color = {0, 0, 127}));
  connect(genrou1.VI, exac21.VI) annotation(Line(points = {{30, -15}, {43.8759, -15}, {43.8759, 5.82424}, {-8.9305, 5.82424}, {-8.9305, 5.82424}}, color = {0, 0, 127}));
  connect(dVREF, exac21.dVREF) annotation(Line(points = {{-80, -8}, {-61.9311, -8}, {-61.9311, -12.2309}, {-39.2165, -12.2309}, {-39.2165, -12.2309}}));
  connect(dGREF, ggov11.dGREF) annotation(Line(points = {{-66, -30}, {-36.8868, -30}, {-36.8868, -25.8208}, {-31.645, -25.8208}, {-31.645, -25.8208}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{15, -30}, {15.6701, -30}, {15.6701, -40.6963}, {26, -40.6963}, {26, -38}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{0, 40}, {14.9166, 40}, {14.9166, 0}, {15, 0}}));
  connect(exac21.EFD0, genrou1.EFD0) annotation(Line(points = {{-10, -15}, {-6, -15}, {-6, -15}, {-0.3, -15}}, color = {0, 0, 127}));
  connect(exac21.EFD, genrou1.EFD) annotation(Line(points = {{-10, -12}, {-6, -12}, {-6, -10.8}, {0, -10.8}}, color = {0, 0, 127}));
  connect(exac21.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-10, -9}, {-6, -9}, {-6, -7.5}, {-0.3, -7.5}}, color = {0, 0, 127}));
  connect(exac21.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-10, -5.1}, {-4, -5.1}, {-4, -3}, {-0.3, -3}}, color = {0, 0, 127}));
  connect(exac21.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-40, 4.5}, {-46, 4.5}, {-46, 20}, {36, 20}, {36, -7.5}, {30, -7.5}}, color = {0, 0, 127}));
  connect(exac21.VUEL, const.y) annotation(Line(points = {{-40, 0}, {-50, 0}, {-50, 6}, {-54.5, 6}}, color = {0, 0, 127}));
  connect(exac21.VOEL, const.y) annotation(Line(points = {{-40, -4.2}, {-50, -4.2}, {-50, 6}, {-54.5, 6}}, color = {0, 0, 127}));
  connect(exac21.VOTHSG, const.y) annotation(Line(points = {{-40, -8.4}, {-50, -8.4}, {-50, 6}, {-54.5, 6}}, color = {0, 0, 127}));
  connect(ggov11.PMECH, genrou1.PMECH) annotation(Line(points = {{-12, -26}, {-8, -26}, {-8, -24}, {0, -24}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {57.0615, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-5.13093, 9.23212}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_EXAC2_GGOV1")}));
end GENROU_EXAC2_GGOV1;
