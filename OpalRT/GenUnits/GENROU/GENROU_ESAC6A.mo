within OpalRT.GenUnits.GENROU;
class GENROU_ESAC6A
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
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TK_ex = 1 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VAMAX_ex = 5 "or zero" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VAMIN_ex = -5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VFELIM_ex = 5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KH_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VHMAX_ex = 5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TH_ex = 0.2 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TJ_ex = 1.2 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KE_ex = 0.5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESAC6A Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {15, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-63,0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC6A esac6a1(TR = TR_ex, KA = KA_ex, TA = TA_ex, TK = TK_ex, TB = TB_ex, TC = TC_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TE = TE_ex, VFELIM = VFELIM_ex, KH = KH_ex, VHMAX = VHMAX_ex, TH = TH_ex, TJ = TJ_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-24,-3}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-71,-9}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(esac6a1.dVREF, dVREF) annotation(Line(points = {{-39, -12}, {
          -68.8525, -12}, {-68.8525, -11.2933}, {-68.8525, -11.2933}},
          color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{0,-27},{
          -2.71399,-27},{-2.71399,-24},{0,-24}}, color = {
          0,0,127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{15,-30},{15,-40.6963},{60,
          -40.6963},{60,-40}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{0,40},{14.9166,40},{
          14.9166,0},{15,0}}));
  connect(esac6a1.EFD0, genrou1.EFD0) annotation (Line(points = {{-9,-15},{-4.5,
          -15},{-4.5,-15},{-0.3,-15}}, color = {0,0,127}));
  connect(esac6a1.EFD, genrou1.EFD) annotation (Line(points = {{-9,-12},{-5,-12},
          {-5,-10.8},{0,-10.8}}, color = {0,0,127}));
  connect(esac6a1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-9,-9},{
          -5,-9},{-5,-7.5},{-0.3,-7.5}}, color = {0,0,127}));
  connect(esac6a1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-9,-5.1},
          {-5,-5.1},{-5,-3},{-0.3,-3}}, color = {0,0,127}));
  connect(genrou1.VI, esac6a1.VI) annotation(Line(points = {{30, -15}, {43.7018, -15},
          {43.7018, 5.9126}, {-8.22622, 5.9126}, {-8.22622, 5.9126}}, color = {0, 0, 127}));
  connect(esac6a1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-39,4.5},
          {-46,4.5},{-46,20},{39,20},{39,-7.5},{30,-7.5}}, color = {0,0,127}));
  connect(const.y, esac6a1.VUEL)
    annotation (Line(points = {{-57.5,0},{-39,0}}, color={0,0,127}));
  connect(esac6a1.VOEL, esac6a1.VUEL) annotation (Line(points = {{-39,-4.2},{
          -48,-4.2},{-48,0},{-39,0}}, color = {0,0,127}));
  connect(esac6a1.VOTHSG, esac6a1.VUEL) annotation (Line(points = {{-39,-8.4},{
          -48,-8.4},{-48,0},{-39,0}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {57.7941, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-5.13093, 9.23212}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_EXAC1A"), Text(origin = {-59.4304, -59.9804}, extent = {{59.0612, -18.9402}, {-20.16, 17.89}}, textString = "dVREF"), Text(origin = {-29.6516, 61.375}, extent = {{59.06, -18.94}, {3.03902, 10.3198}}, textString = "TRIP")}));
end GENROU_ESAC6A;
