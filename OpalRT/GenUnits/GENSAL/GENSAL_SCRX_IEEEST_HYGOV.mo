within OpalRT.GenUnits.GENSAL;
class GENSAL_SCRX_IEEEST_HYGOV
 parameter Real partType = 1;
  // GENSAL Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  // HYGOV Parameters
  parameter Real R_tg = 0.06 "Permanent Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real r_tg = 0.4 "Temporary Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tr_tg = 8 "(>0) Governor time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tf_tg = 0.05 "(>0) Filter time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tg_tg = 0.2 "(>0) Servo time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real VELM_tg = 0.01 "Gate velocity limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMAX_tg = 0.601 "Maximum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMIN_tg = 0 "Minimum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real TW_tg = 1.2 "(>0) Water time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real At_tg = 2.5 "Trubine gain" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Dturb_tg = 0 "Turbine damping" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real qNL_tg = 0.5 "No power flow" annotation(Dialog(tab = "HYGOV Parameters"));
  //
  // IEEEST Parameters
  parameter Real A1_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
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
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV hygov1(R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, VELM = VELM_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, At = At_tg, Dturb = Dturb_tg, qNL = qNL_tg, IBUS = IBUS, ID = ID) annotation(Placement(visible = true, transformation(origin = {12, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-16, 32}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(IBUS = IBUS, ID = ID, TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, CSWITCH = CSWITCH_ex, rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin = {12, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin = {-22, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-24, -22}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-53, -10}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-24, -10}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(gensal1.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{50, -2.6}, {57.4658, -2.6}, {57.4658, -30.8685}, {-38.0517, -30.8685}, {-38.0517, 1.94141}, {-32.2275, 1.94141}, {-32.2275, 1.94141}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{50, -8}, {55.9127, -8}, {55.9127, -27.9563}, {-38.0517, -27.9563}, {-38.0517, 1.94141}, {-31.8392, 1.94141}, {-31.8392, 1.94141}}, color = {0, 0, 127}));
  connect(gensal1.VI, ieeest1.VI2) annotation(Line(points = {{50, 0}, {58.0482, 0}, {58.0482, -32.6157}, {-35.5279, -32.6157}, {-35.5279, 3.88283}, {-31.8392, 3.88283}, {-31.8392, 3.88283}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-32, 4}, {-35.3337, 4}, {-35.3337, 9.90121}, {-32.0333, 9.90121}, {-32.0333, 9.90121}}, color = {0, 0, 127}));
  connect(dVREF, scrx1.dVREF) annotation(Line(points = {{-24, -10}, {-12.8133, -10}, {-12.8133, -2.71798}, {-2.13555, -2.71798}, {-2.13555, 1.55313}, {2.13555, 1.55313}, {2.13555, 1.55313}}));
  connect(dGREF, hygov1.dGREF) annotation(Line(points = {{-24, -22}, {-8.34808, -22}, {-8.34808, -6.6008}, {2.13555, -6.6008}, {2.13555, -6.6008}}));
  connect(gensal1.VI, hygov1.VI) annotation(Line(points = {{50, 0}, {58.2424, 0}, {58.2424, -32.6157}, {-4.85353, -32.6157}, {-4.85353, -13.784}, {2.3297, -13.784}, {2.3297, -13.784}}, color = {0, 0, 127}));
  connect(gensal1.MBASE, hygov1.MBASE) annotation(Line(points = {{50, -5.4}, {56.6893, -5.4}, {56.6893, -29.5095}, {-2.52384, -29.5095}, {-2.52384, -18.0551}, {2.13555, -18.0551}, {2.13555, -18.0551}}, color = {0, 0, 127}));
  connect(scrx1.VI, gensal1.VI) annotation(Line(points = {{22, 14}, {58.0482, 14}, {58.0482, 0}, {50.4767, 0}, {50.4767, 0}}, color = {0, 0, 127}));
  connect(TRIP, gensal1.TRIP) annotation(Line(points = {{40, 35}, {40.1656, 35}, {40.1656, 10}, {40, 10}}));
  //
  connect(gensal1.p, bus0) annotation(Line(points = {{40, -10}, {76, -10}, {76, -10}, {80, -10}}));
  connect(scrx1.EFD0, gensal1.EFD0) annotation(Line(points = {{22, 0}, {29.8, 0}}, color = {0, 0, 127}));
  connect(scrx1.EFD, gensal1.EFD) annotation(Line(points = {{22, 2}, {26, 2}, {26, 2.8}, {30, 2.8}}, color = {0, 0, 127}));
  connect(scrx1.ETERM0, gensal1.ETERM0) annotation(Line(points = {{22, 4}, {26, 4}, {26, 5}, {29.8, 5}}, color = {0, 0, 127}));
  connect(scrx1.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{22, 6.6}, {26, 6.6}, {26, 8}, {29.8, 8}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, scrx1.VOTHSG) annotation(Line(points = {{-12, 2}, {-6, 2}, {-6, 4.4}, {2, 4.4}}, color = {0, 0, 127}));
  connect(gensal1.XADIFD, scrx1.XADIFD) annotation(Line(points = {{50, 5}, {56, 5}, {56, 22}, {-4, 22}, {-4, 13}, {2, 13}}, color = {0, 0, 127}));
  connect(scrx1.VUEL, const.y) annotation(Line(points = {{2, 10}, {-6, 10}, {-6, 32}, {-10.5, 32}}, color = {0, 0, 127}));
  connect(scrx1.VOEL, const.y) annotation(Line(points = {{2, 7.2}, {-6, 7.2}, {-6, 32}, {-10.5, 32}}, color = {0, 0, 127}));
  connect(hygov1.PMECH, gensal1.PMECH) annotation(Line(points = {{22, -6}, {24, -6}, {24, -6}, {30, -6}}, color = {0, 0, 127}));
  connect(gensal1.PMECH0, hygov1.PMECH0) annotation(Line(points = {{30, -8}, {26, -8}, {26, -26}, {22, -26}, {22, -8}}, color = {0, 0, 127}));
  connect(hygov1.SLIP, gensal1.SLIP) annotation(Line(points = {{2, -22}, {-2, -22}, {-2, -28}, {56, -28}, {56, -8}, {50, -8}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation(Line(points = {{-31.8, 2}, {-38, 2}, {-38, 8}, {-32, 8}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-57.06, 28.58}, extent = {{143.62, -49.31}, {-24.72, 14.46}}, textString = "GENSAL_SCRX_IEEEST_HYGOV"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAL_SCRX_IEEEST_HYGOV;
