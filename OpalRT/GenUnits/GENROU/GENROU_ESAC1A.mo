within OpalRT.GenUnits.GENROU;
class GENROU_ESAC1A
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
  // ESAC6A Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VAMAX_ex = 5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VAMIN_ex = -5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KF_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TF_ex = 0.2 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KE_ex = 0.5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VRMAX_ex = 5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "ESAC1A Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {15, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-64,-8}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC1A esac1a1(TR = TR_ex, TB = TB_ex, TC = TC_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex) annotation(Placement(visible = true, transformation(origin={-25,-3}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-75,-18}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-80, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant constant1(k = noVOEL) annotation(Placement(visible = true, transformation(origin={-64,7}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = noVUEL) annotation(Placement(visible = true, transformation(origin={-64,23}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = -Modelica.Constants.inf;
equation
  connect(esac1a1.dVREF, dVREF) annotation(Line(points = {{-40, -12}, {
          -54.2429, -12}, {-54.2429, -17.97}, {-70.5491, -17.97}, {-70.5491,
          -17.97}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{0,-27},{
          -2.71399,-27},{-2.71399,-24},{0,-24}}, color = {
          0,0,127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{15,-30},{15,-40.6963},{60,
          -40.6963},{60,-40}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{0,40},{14.9166,40},{
          14.9166,0},{15,0}}));
  connect(esac1a1.EFD0, genrou1.EFD0)
    annotation (Line(points = {{-10,-15},{-0.3,-15}}, color={0,0,127}));
  connect(esac1a1.EFD, genrou1.EFD) annotation (Line(points = {{-10,-12},{-5,
          -12},{-5,-10.8},{0,-10.8}}, color = {0,0,127}));
  connect(esac1a1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-10,-9},{
          -5,-9},{-5,-7.5},{-0.3,-7.5}}, color = {0,0,127}));
  connect(esac1a1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-10,-5.1},
          {-5.5,-5.1},{-5.5,-3},{-0.3,-3}}, color = {0,0,127}));
  connect(genrou1.VI, esac1a1.VI) annotation(Line(points = {{30, -15}, {40.1028, -15},
          {40.1028, 6.16967}, {-9.51157, 6.16967}, {-9.51157, 6.16967}}, color = {0, 0, 127}));
  connect(esac1a1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-40,4.5},
          {-47,4.5},{-47,18},{36,18},{36,-7.5},{30,-7.5}}, color = {0,0,127}));
  connect(const.y, esac1a1.VOTHSG) annotation (Line(points = {{-58.5,-8},{-49,
          -8},{-49,-8.4},{-40,-8.4}}, color = {0,0,127}));
  connect(esac1a1.VOEL, constant1.y) annotation (Line(points = {{-40,-4.2},{-56,
          -4.2},{-56,7},{-58.5,7}}, color = {0,0,127}));
  connect(esac1a1.VUEL, constant2.y) annotation (Line(points = {{-40,0},{-53,0},
          {-53,23},{-58.5,23}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {81.3394, -65.6766}, extent = {{4.19912, -2.19976}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-17.8508, 8.14958}, extent = {{-73.46, 18.79}, {105.862, -43.6543}}, textString = "GENROU_EXAC1A"), Text(origin = {-45.8986, 48.5446}, extent = {{10.3467, 5.1464}, {-20.16, 17.89}}, textString = "dVREF"), Text(origin = {-3.67054, 68.6822}, extent = {{44.9869, -7.03202}, {3.03902, 10.3198}}, textString = "TRIP")}));
end GENROU_ESAC1A;
