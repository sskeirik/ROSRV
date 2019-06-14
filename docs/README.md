# Introduction

The purpose of this document is to -

1)  Provide a brief theoretical background behind three distinct
    projects that are used in unison for Runtime Verification (RV) of
    ROS based application -
    
      - [ROSRV](https://github.com/Formal-Systems-Laboratory/ROSRV/),
      - [RV-Monitor](https://github.com/runtimeverification/rv-monitor)
        and
      - [ROSMOP](https://github.com/Formal-Systems-Laboratory/rosmop)

2)  Discuss the *overall architecture* needed for RV of ROS applications
    using aforementioned tools. The tools are maintained in separate
    repositories for modularity. Each tool can be used independently,
    but most use cases require tight interaction. We describe our
    current development and testing setup that allows us the advantages
    of having a “mono-repo” like setup.

3)  Layout guidelines for -
    
      - Contributions to each of the codebase, or to the fundamental
        theory behind them.
      - Setting up appropriate RV infrastructure based on need.

4)  Describe the current status of development, immediate goals, and a
    roadmap for long term goals.

# Preliminaries

## Monitoring Oriented Programming (MOP)

In essence MOP is a framework that uses runtime monitoring as a
fundamental principle for building reliable systems \[1\]. (Please see
<http://fsl.cs.illinois.edu/index.php/MOP_Papers> for a complete list of
paper on MOP). At the core of MOP is the idea that all monitoring
systems share certain functionality, mainly program instrumentation,
monitor integration and monitor generation. Identifying and isolating
different functionality allows MOP to be instantiantiated with diffrent
programming languages and logical formalisms.

### MOP Architecture

There are two types of high level components in MOP -

1)  *Language Client* - The language client is responsible for “language
    specific” functionality, and hides the logic internals of the logic
    repository from from the user. Since language clients are coupled to
    a language, they are ocasionally referred to by the name of the MOP
    instance to which they belong.

2)  *Logic Repository* - The *logic repository* consists of *logic
    plugins*. Each plugin encapsulates a monitor synthesis algorithm for
    a particular specification formalism. In \[1\], the authors describe
    in detail many plugins, but for the purpose of this document, we’ll
    use the Differential Dynamic Logic plugin - the most recent, and for
    the purpose of monitoring ROS applications, a relevant plugin, as a
    motivating example throughtout the rest of this document. The output
    of a logic plugin is generally pseudo-code, which is used by the
    language client to generate concrete monitoring code in the taget
    language. This makes the process of monitor generation language
    independent.

### Example MOP Instantiaitions

#### JavaMOP

JavaMOP is an instantiation of the MOP framework for Java \[2\].
Effecient monitor generation algorithms, \[3\], \[4\] are employed in
logic plugins to generate language independent pseudo code. JavaMOP uses
language independent output of aforementioned logic plugins, and
synthesis AspectJ code \[5\], which is weaved into the targe application
with any apsectJ compiler.

#### CMOP

As suggested by the name, CMOP is an MOP instantiation for C
applications. Due to the low level nature of C, it finds applications in
Cyber Physical System (CPS) like robots and autonomous cars. The
Differential Dynamic Logic (dL) plugin is a recent addition to the MOP
logic repository with the intention of making it easier to express
specifications involving Hybrid Systems. Please see
<http://symbolaris.com/logic/dL.html> for more information about the
syntax and semantics of the logic. A short presentation about dL by the
author of this document can be found at
<https://github.com/msaxena2/logic-types-notes/blob/master/differential-dynamic-logic/fm-seminar-talk.pdf>.
Currently, we depend on [KeymaeraX](http://www.ls.cs.cmu.edu/KeYmaeraX/)
\[6\], a dL theorem prover for parising and reasoning about differenial
dynamic logic formulas, and on ModelPlex \[7\], a tool built on top of
KeymaeraX to generate monitoring code. For instrumentation, the C plugin
relies on [LLVM](https://llvm.org) \[8\]

#### ROSMOP

ROSMOP is an instance of MOP for the widely used Robot Operating System
(ROS). one of ROSMOP’s main strength’s is the frontend-syntax of it’s
language client. ROSMOP allows declaration of events (with arguments),
where the user writes *patterns* to bind relevant parts of a message’s
arguments to the event. For instance, consider the following
specification -

``` c

event aEvent(double eventPar)  /test_channel
                                std_msgs/Float64 {data : eventPar}
{
  ROS_INFO(test);
}

```

The pattern `{data : eventPar}` specifies that for the monitored channel
`/test_channel`, with message of type `std_msgs::Float64`, the double
eventPar in the event should be bound to the `data` of the message. This
specification style makes ROSMOP an extremely effective tool for
monitoring ROS applications. Thus, ROSMOP has been used primarily in
“raw” specifications, or specifications without a logical formulism,
where code is specified in the event body itself. Combined with
(RORSRV)\[http://fsl.cs.illinois.edu/index.php/ROSRV\] \[9\], ROSMOP
expressiveness has been sufficient for non trivial applications \[10\].

# Architecture & Layout

In this section, we primarily focus on -

1)  The mapping of theory laid out in to each of the tools. We discuss,
    for each tool, how the implementation complies with the MOP
    architecture discussed in \[1\].

2)  We go into engineering details of the codebase’s organization,
    interaction between them and the setup to maintain each repository
    individually, but have advantages “mono-repo” like setup.

The following sections discuss, for each tool, the theory behind them.

### RV-Monitor

RV-Monitor is an open source tool developed and maintained by [Runtime
Verification Inc](https://runtimeverification.com). The source is
available <https://github.com/runtimeverification/rv-monitor>.
RV-Monitor is a comprehensive tool for monitoring Java and C. It is an
implementation of the MOP architecture - it has language clients for
Java and C, and its logic repository contains multiple specification
languages. In essence, RV-Monitor contains both JavaMOP, and CMOP (also
called RV-Monitor for C).

RV-Monitor’s logic repository a comprehensive list of logic plugins.
Since the number of plugins keeps growing, we refer the reader to the
<https://github.com/runtimeverification/rv-monitor/tree/master/plugins_logicrepository>,
the part of the codebase where logic plugins are maintained.

To understand a logic plugin’s code in greater detail, consider the dL
plugin at
<https://github.com/runtimeverification/rv-monitor/tree/master/plugins_logicrepository>.
The plugin’s main logic is an implementation `CDL.java` of the abstract
class `LogicPluginShell.java`. LogicPluginShell is a wrapper around each
Logic Plugin in the repository, allowing the language clients access to
the Plugins.
<https://github.com/runtimeverification/rv-monitor/tree/master/plugins_logicrepository>.

The class `DLPlugin.java`\[1\] is an implementation of the abstract
class `LogicPlugin.java`.\[2\] Each implementation of `LogicPlugin.java`
is encapsulates the monitor generation algorithm. As the name implies,
it’s an implementation (as a programming contract) `LogicPlugin` layer
of the MOP stack. The class specifies the functionality that each plugin
must implement to fit into RV-Monitor’s strict MOP based architecture.

### ROSRV Architecture

Rosrv has a nice architecture.

## References

<div id="refs" class="references">

<div id="ref-MOP2012">

\[1\] G. Roşu and F. Chen, “Semantics and algorithms for parametric
monitoring,” *Logical Methods in Computer Science*, vol. 8, no. 1, pp.
1–47, 2012. 

</div>

<div id="ref-JavaMOP2012">

\[2\] D. Jin, P. O. Meredith, C. Lee, and G. Roşu, “JavaMOP: Efficient
parametric runtime monitoring framework,” in *Proceeding of the 34th
international conference on software engineering (icse’12)*, 2012, pp.
1427–1430. 

</div>

<div id="ref-ParametricCFP2008">

\[3\] P. Meredith, D. Jin, F. Chen, and G. Roşu, “Efficient monitoring
of parametric context-free patterns,” in *Proceedings of the 23rd
ieee/acm international conference on automated software engineering(ASE
’08)*, 2008, pp. 148–157. 

</div>

<div id="ref-DynamicProgrammingLTL2001">

\[4\] G. Rosu and K. Havelund, “Synthesizing dynamic programming
algorithms from linear temporal logic formulae,” Research Institute for
Advanced Computer Science,
https://ti.arc.nasa.gov/m/pub-archive/archive/0220.pdf, 2001. 

</div>

<div id="ref-AspectJUrl">

\[5\] “AspectJ.” https://www.eclipse.org/aspectj. 

</div>

<div id="ref-KeymaeraXUrl">

\[6\] “KeYmaera x: An aXiomatic tactical theorem prover for hybrid
systems.” https://ls.cs.cmu.edu/KeYmaeraX/. 

</div>

<div id="ref-ModelPlex2016">

\[7\] S. Mitsch and A. Platzer, “ModelPlex: Verified runtime validation
of verified cyber-physical system models,” *Formal Methods in System
Design*, vol. 49, no. 1, pp. 33–74, Oct. 2016 \[Online\]. Available:
<https://doi.org/10.1007/s10703-016-0241-z>

</div>

<div id="ref-LLVMUrl">

\[8\] “The llvm compiler infrastructure.” https://llvm.org. 

</div>

<div id="ref-ROSRVUrl">

\[9\] “ROSRV.” https://fsl.cs.illinois.edu/index.php/ROSRV. 

</div>

<div id="ref-ROSRV16">

\[10\] J. Huang *et al.*, “ROSRV: Runtime verification for robots,” in
*Proceedings of the 14th international conference on runtime
verification*, 2014, vol. 8734, pp. 247–254. 

</div>

</div>

1.  <https://github.com/runtimeverification/rv-monitor/blob/master/plugins_logicrepository/dl/src/main/java/com/runtimeverification/rvmonitor/logicrepository/plugins/dl/DLPlugin.java>

2.  <https://github.com/runtimeverification/rv-monitor/blob/master/logicrepository/src/main/java/com/runtimeverification/rvmonitor/logicrepository/plugins/LogicPlugin.java>
