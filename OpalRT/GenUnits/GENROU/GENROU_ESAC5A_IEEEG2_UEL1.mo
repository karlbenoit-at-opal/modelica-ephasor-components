within OpalRT.GenUnits.GENROU;
class GENROU_ESAC5A_IEEEG2_UEL1
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
  // ESAC5A Parameters
  parameter Real TR_ex = 0.02 "(sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real KA_ex = 100 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TA_ex = 0.5 "(sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real VRMIN_ex = -5 "V" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TF2_ex = 2 "(>0) (sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TF3_ex = 2 "(>0) (sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESAC5A Parameters"));
  // UEL1 Parameters
  parameter Real KUR_uel = 2 "UEL radius setting (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUC_uel = 1 "(pu) UEL center setting (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUF_uel = 1 "(pu) UEL excitation system stabilizergain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VURMAX_uel = 1 "UEL maximum limit for radius phasor magnitude (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUCMAX_uel = 1 "UEL maximum limit for operating point phasor magnitude (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUI_uel = 1 "UEL integral gain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUL_uel = 1 "UEL proportional gain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUIMAX_uel = 1 "UEL integrator output maximum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUIMIN_uel = 1 "UEL integrator output minimum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU1_uel = 1 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU2_uel = 1 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU3_uel = 1 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU4_uel = 1 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VULMAX_uel = 1 "UEL output maximum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VULMIN_uel = 1 "UEL output minimum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  //ieeeg2
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "IEEEG2"));
  parameter Real K_tg = 20 "K" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T1_tg = 20 "T1" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T2_tg = 20 "T2" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T3_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMAX_tg = 20 "PMAX (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMIN_tg = 20 "PMIN (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T4_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-70,30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL1 uel11(KUR = KUR_uel, KUC = KUC_uel, KUF = KUF_uel, VURMAX = VURMAX_uel, VUCMAX = VUCMAX_uel, KUI = KUI_uel, KUL = KUL_uel, VUIMAX = VUIMAX_uel, VUIMIN = VUIMIN_uel, TU1 = TU1_uel, TU2 = TU2_uel, TU3 = TU3_uel, TU4 = TU4_uel, VULMAX = VULMAX_uel, VULMIN = VULMIN_uel) annotation(Placement(visible = true, transformation(origin={-76,0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-98,-26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG2 ieeeg21(K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg) annotation(Placement(visible = true, transformation(origin={0,-48}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC5A esac5a1(TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, TF3 = TF3_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-6,0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={53,45}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin={-51.5,-36}, extent={{-6.5,-6},{6.5,6}}, rotation = 0), iconTransformation(origin={-100,-60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus0, genrou1.p) annotation(Line(points = {{100,-80},{60,-80},{60,-45}}));
  connect(esac5a1.EFD0, genrou1.EFD0)
    annotation (Line(points = {{19,-20},{34.5,-20}}, color={0,0,127}));
  connect(esac5a1.EFD, genrou1.EFD) annotation (Line(points = {{19,-15},{26.5,
          -15},{26.5,-13},{35,-13}}, color = {0,0,127}));
  connect(esac5a1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{19,-10},{
          26,-10},{26,-7.5},{34.5,-7.5}}, color = {0,0,127}));
  connect(esac5a1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{19,-3.5},
          {26.5,-3.5},{26.5,0},{34.5,0}}, color = {0,0,127}));
  connect(esac5a1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-31,12.5},
          {-42,12.5},{-42,38},{98,38},{98,-7.5},{85,-7.5}}, color = {0,0,127}));
  connect(ieeeg21.PMECH, genrou1.PMECH) annotation (Line(points = {{15,-36},{20,
          -36},{20,-35},{35,-35}}, color = {0,0,127}));
  connect(genrou1.PMECH0, ieeeg21.PMECH0) annotation (Line(points={{35,-40},{20,
          -40},{20,-39},{15,-39}}, color = {0,0,127}));
  connect(uel11.VUEL, esac5a1.VUEL) annotation (Line(points = {{-60.4,0},{-52,0},
          {-52,5},{-31,5}}, color = {0,0,127}));
  connect(genrou1.SLIP, ieeeg21.SLIP) annotation (Line(points = {{85,-40},{88,
          -40},{88,-72},{-24,-72},{-24,-60},{-15,-60}}, color = {0,0,127}));
  connect(const.y, esac5a1.VOEL) annotation (Line(points = {{-59,30},{-46,30},{
          -46,-2},{-31,-2}}, color = {0,0,127}));
  connect(esac5a1.VOTHSG, esac5a1.VOEL) annotation (Line(points = {{-31,-9},{
          -46,-9},{-46,-2},{-31,-2}}, color = {0,0,127}));
  connect(uel11.VF, esac5a1.VF) annotation (Line(points = {{-90.7,0},{-100,0},{
          -100,52},{26,52},{26,20},{20,20}}, color = {0,0,127}));
  connect(uel11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-90.7,12},{
          -94,12},{-94,46},{30,46},{30,0},{34.5,0}}, color = {0,0,127}));
  connect(genrou1.TRIP, TRIP) annotation (Line(points = {{60,5},{62,5},{62,45},
          {53,45}}, color = {0,0,127}));
  connect(dVREF, esac5a1.dVREF) annotation (Line(points={{-98,-26},{-40,-26},{-40,
          -15},{-31,-15}}, color={0,0,0}));
  connect(genrou1.VI, esac5a1.VI) annotation (Line(points={{85,-20},{92,-20},{92,
          15},{19,15}}, color={0,0,127}));
  connect(genrou1.VI, ieeeg21.VI) annotation (Line(points={{85,-20},{96,-20},{96,
          -68},{-22,-68},{-22,-48},{-15,-48}}, color={0,0,127}));
  connect(genrou1.MBASE, ieeeg21.MBASE) annotation (Line(points={{85,-33.5},{92,
          -33.5},{92,-64},{20,-64},{20,-76},{-18,-76},{-18,-54},{-15,-54}},
        color={0,0,127}));
  connect(dGREF, ieeeg21.dGREF)
    annotation (Line(points={{-51.5,-36},{-15,-36}}, color={0,0,0}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-81.55, 7.17}, extent = {{-12.53, 6.49}, {175.86, -14.01}}, textString = "GENROU_ESAC5A_IEEEG2_UEL1"), Rectangle(origin = {0.113895, -0.341686}, extent = {{-99.8861, 99.8861}, {99.8861, -99.8861}})}));
end GENROU_ESAC5A_IEEEG2_UEL1;
