within OpalRT.GenUnits.GENROU;
class GENROU_EXAC1_IEEEG2_UEL1
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
  // EXAC1 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real TF_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXAC1 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXAC1 Parameters"));
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
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG2 ieeeg21(K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg) annotation(Placement(visible = true, transformation(origin={0,-62}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL1 uel11(
    KUR = KUR_uel,
    KUC = KUC_uel,
    KUF = KUF_uel,
    VURMAX = VURMAX_uel,
    VUCMAX = VUCMAX_uel,
    KUI = KUI_uel,
    KUL = KUL_uel,
    VUIMAX = VUIMAX_uel,
    VUIMIN = VUIMIN_uel,
    TU1 = TU1_uel,
    TU2 = TU2_uel,
    TU3 = TU3_uel,
    TU4 = TU4_uel,
    VULMAX = VULMAX_uel,
    VULMIN = VULMIN_uel) annotation(Placement(visible = true, transformation(origin={-76,-8}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(
    ID = M_ID,
    P_gen = P_gen,
    Q_gen = Q_gen,
    Vt_abs = Vt_abs,
    Vt_ang = Vt_ang,
    SB = SB,
    fn = fn,
    ZSOURCE_RE = ZSOURCE_RE,
    Tdo_p = Tdo_p,
    Tdo_s = Tdo_s,
    Tqo_p = Tqo_p,
    Tqo_s = Tqo_s,
    H = H,
    D = D,
    Xd = Xd,
    Xq = Xq,
    Xd_p = Xd_p,
    Xq_p = Xq_p,
    Xd_s = Xd_s,
    Xl = Xl,
    S1 = S1,
    S12 = S12) annotation(Placement(visible = true, transformation(origin={60,-36}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXAC1 exac11(
    ID = M_ID,
    TR = TR_ex,
    TB = TB_ex,
    TC = TC_ex,
    KA = KA_ex,
    TA = TA_ex,
    VRMAX = VRMAX_ex,
    VRMIN = VRMIN_ex,
    TE = TE_ex,
    KF = KF_ex,
    TF = TF_ex,
    KC = KC_ex,
    KD = KD_ex,
    KE = KE_ex,
    E1 = E1_ex,
    SE_E1 = SE_E1_ex,
    E2 = E2_ex,
    SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-2,-16}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-66,42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={100,-96}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin={100,-96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-96,-38}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation (Placement(visible=true, transformation(origin={-96,-72}, extent={{-10,-10},{10,10}}, rotation=0), iconTransformation( origin={-100,-40}, extent={{-10,-10},{10,10}}, rotation=0)));
  OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={60,34}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation 
  connect(TRIP,genrou1. TRIP) annotation(Line(points = {{60,34},{60.0473,34},{
          60.0473,-11},{60,-11}}));
  connect(bus0,genrou1. p) annotation(Line(points = {{100,-96},{61.0495,-96},{
          61.0495,-68.1499},{60,-68.1499},{60,-61}}));
  connect(exac11.EFD0, genrou1.EFD0)
    annotation (Line(points = {{23,-36},{34.5,-36}}, color={0,0,127}));
  connect(exac11.EFD, genrou1.EFD) annotation (Line(points = {{23,-31},{29.5,
          -31},{29.5,-29},{35,-29}}, color = {0,0,127}));
  connect(exac11.ETERM0, genrou1.ETERM0) annotation (Line(points = {{23,-26},{
          30,-26},{30,-23.5},{34.5,-23.5}}, color = {0,0,127}));
  connect(exac11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{23,-19.5},
          {28.5,-19.5},{28.5,-16},{34.5,-16}}, color = {0,0,127}));
  connect(exac11.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-27,-3.5},
          {-38,-3.5},{-38,20},{94,20},{94,-23.5},{85,-23.5}}, color = {0,0,127}));
  connect(const.y, exac11.VOEL) annotation (Line(points = {{-55,42},{-44,42},{
          -44,-18},{-27,-18}}, color = {0,0,127}));
  connect(exac11.VOTHSG, exac11.VOEL) annotation (Line(points = {{-27,-25},{-44,
          -25},{-44,-18},{-27,-18}}, color = {0,0,127}));
  connect(exac11.VUEL, uel11.VUEL) annotation (Line(points = {{-27,-11},{-52,
          -11},{-52,-8},{-60.4,-8}},
                                   color = {0,0,127}));
  connect(uel11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-90.7,4},{
          -98,4},{-98,24},{30,24},{30,-16},{34.5,-16}}, color = {0,0,127}));
  connect(uel11.VF, exac11.VF) annotation (Line(points = {{-90.7,-8},{-102,-8},
          {-102,26},{28,26},{28,4},{24,4}},color = {0,0,127}));
  connect(ieeeg21.PMECH, genrou1.PMECH) annotation (Line(points = {{15,-50},{20,
          -50},{20,-51},{35,-51}}, color = {0,0,127}));
  connect(ieeeg21.PMECH0, genrou1.PMECH0) annotation (Line(points = {{15,-53},{22,
          -53},{22,-56},{35,-56}}, color = {0,0,127}));
  connect(ieeeg21.SLIP, genrou1.SLIP) annotation (Line(points={{-15,-74},{-20,-74},
          {-20,-80},{90,-80},{90,-56},{85,-56}}, color = {0,0,127}));
  connect(genrou1.VI, uel11.VI) annotation (Line(points = {{85,-36},{90,-36},{90,14},
          {-96,14},{-96,-2},{-90.7,-2}}, color = {0,0,127}));
  connect(dVREF, exac11.dVREF) annotation (Line(points = {{-96,-38},{-40,-38},{-40,
          -31},{-27,-31}}, color = {0,0,0}));
  connect(genrou1.MBASE, ieeeg21.MBASE) annotation (Line(points = {{85,-49.5},{92,
          -49.5},{92,-82},{-24,-82},{-24,-68},{-15,-68}}, color = {0,0,127}));
  connect(genrou1.VI, ieeeg21.VI) annotation (Line(points = {{85,-36},{94,-36},{94,
          -84},{-28,-84},{-28,-62},{-15,-62}}, color = {0,0,127}));
  connect(exac11.VI, genrou1.VI) annotation (Line(points = {{23,-1},{98,-1},{98,-36},
          {85,-36}}, color = {0,0,127}));
  connect(dGREF, ieeeg21.dGREF) annotation (Line(points={{-96,-72},{-40,-72},{-40,
          -50},{-15,-50}}, color={0,0,0}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-81.547, 7.16964}, extent = {{-12.53, 6.49}, {175.856, -14.0071}}, textString = "GENROU_EXAC1_IEEEG2_UEL1"), Rectangle(origin = {0.113895, -0.341686}, extent = {{-99.8861, 99.8861}, {99.8861, -99.8861}})}));
end GENROU_EXAC1_IEEEG2_UEL1;
