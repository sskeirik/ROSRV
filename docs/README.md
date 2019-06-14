# Introduction

The purpose of this document is to -

1)  Provide a brief theoretical background behind three projects that
    are used in unison for Runtime Verification (RV) of ROS based
    application -
    
      - [ROSRV](https://github.com/Formal-Systems-Laboratory/ROSRV/)
      - [RV-Monitor](https://github.com/runtimeverification/rv-monitor)
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
different functionality allows MOP to be instantiated with different
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

### MOP Instances

#### JavaMOP

JavaMOP is an instantiation of the MOP framework for Java \[2\].
Efficient monitor generation algorithms, \[3\], \[4\] are employed in
logic plugins to generate language independent pseudo code for
monitoring. JavaMOP uses language independent output of aforementioned
logic plugins, and synthesis AspectJ code \[5\], which is weaved into
the target application with any apsectJ compiler.

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
\[6\], a dL theorem prover for parsing and reasoning about differenial
dynamic logic formulas, and on ModelPlex \[7\], a tool built on top of
KeymaeraX to generate monitoring code. For instrumentation, the C plugin
relies on [LLVM](https://llvm.org) \[8\]

#### ROSMOP

ROSMOP is an instance of MOP for the widely used Robot Operating System
(ROS). One of ROSMOP’s main strength’s is the frontend-syntax of it’s
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

2)  We go into engineering details of the codebase’s components,
    interaction between said components and the setup to maintain each
    repository individually, but to also have advantages “mono-repo”
    like advantages.

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

RV-Monitor, though maintained by Runtime Verification Inc. (RV Inc.), is
open source under the UIUC/NCSA license and encourages contributions
from the community. The codebase is a mix of Java and Scala, and is
managed via a standard [maven](https://maven.apache.org/) \[11\] setup,
i.e. the repository structure is dictated by maven.

### ROSRV

ROSRV is an open source tool for comprehensive runtime monitoring of ROS
based applications, and is maintained by the Formal Systems Laboratory
at UIUC. The source is available at
<https://github.com/Formal-Systems-Laboratory/ROSRV> under the UIUC/NCSA
open source license. At the core of ROSRV is the idea of `RVMaster`, a
special ROS node that serves as an intermediate layer between
`ROSMaster` and other ROS nodes \[10\]. ROSRV relies on a `policy` file
to start generate the `ROSMaster` node. Please see
<https://github.com/Formal-Systems-Laboratory/ROSRV/blob/master/docs/Usage.md>
for more details on configuring ROSRV.

ROSRV is maintained as a `catkin`\[3\] project - the standard way of
maintaining and distributing ROS packages. For users interested in
contributing to ROSRV, basic familiarity with the `catkin` build system
may be helpful.

ROSRV conceptually is not concerned with logical specification and
monitor generation. As long the monitoring logic is executable code
expressed in C++ using standard ROSCpp libraries, ROSRV can use it in
conjunction with RVMaster to critical functionality like preventing a
node from sending a message to another node if the message violates the
monitoring logic. ROSRV does this since it essentially replaces
ROSmaster, providing but maintains much finer grained control over the
overall messaging infrastructure, to the point where it can intervene
and stop messages from reaching target nodes if monitoring violations
are detected.

Practically, manually generating C++ code for any practical monitoring
use is tedious and error prone. Thus, ROSRV relies heavily on ROSMOP for
specification of monitoring logic and monitor generation. Thus, ROSMOP
is used as a git submodule within ROSRV.

### ROSMOP

As the name suggests, ROSMOP is simply an instance of MOP for ROS.
However, ROSMOP had until recently, mainly used for parsing raw
specifications and generating monitors where ROS code is specified
within the event itself, which is in turn used by ROSRV as the
monitoring code along with ROSMaster.

With the introduction of the dL logic plugin in RV-Monitor, with the
ability to generate C code for dL specifications lead to the following
realizations -

1)  dL’s use as a logical formalism deals with Hybrid Systems. This is
    intentional - as a variant of First Order Dynamic Logic \[12\]
    introduces operators for continuous evolution of state variables as
    an extension of Dynamic Programs. This extension of Dynamic
    Programs, known as Hybrid Programs, is how Hybrid Systems are
    modeled in dL. The necessity/possibility operators, (also referred
    to in temporal logics as always/eventually) are used over hybrid
    programs to create formulas. These formulas are used as logical
    specifications in RV-Monitor’s dL plugin. The nature of the
    development of Hybrid Systems is often as follows -
    
    1)  A model is described generally, let’s say in dL, as
        `{Ctrl;Plant}*`. Briefly, the `Ctrl` is a controller for the
        Hybrid System, and the plant is a model of the environment
        dynamics. The same model can be, in most cases, also expressed
        via a Hybrid Automaton, where, naively edges describe discrete
        controller actions, and nodes describe continous evolution as a
        model of the environment dynamics.
    
    2)  A controller is synthesized from a specification like one above,
        or handwritten in a way that it’s faithful to the behavior
        described in its model.
    
    3)  The controller than needs to be tested. Usually, this involves
        the use of a simulator that mimics the continous dynamics
        described in the model. For instance, simulators for controllers
        of Hybrid Systems like cars often use game engines since they
        can provide, to a certain degree, a good approximation of the
        real world physics that the continous dynamics described in the
        plant intend to capture. Purpose built simulators with physics
        engines that try to mimic real world physics as accurately as
        possible are also often employed. This simulation based
        development approach is pretty standard, as it allows fine
        tuning of the controller enriching the model to get intended
        results. For Learning Based Approaches, simulators are used to
        train controller components before they’re used.
    
    4)  Once the simulation is done, the next step is to test the
        controller on the platform itself. Sometimes an intermediate
        step, such as using a smaller/lighter and cheaper platform may
        be performed as it allows for the controller to be exposed to
        real world physics, and any failures while working with a
        smaller platform bear significantly less risks. Finally, the
        controller is put to use on the platform.

Since simulations are often digital, they can be monitored using a tool
like RV-Monitor, which can instrument, for instance the LLVM binary of
the simulation with monitors. Testing/Deployment platforms on the other
hand often use ROS, for which a tool like ROSRV can be employed.

2)  At times, the complete ROSRV infrastructure is too intrusive. It was
    realized that making ROSMOP an independent unit, that can not only
    generate C code for dL specifications, but also standalone ROS nodes
    that can be run independently can significantly help with monitoring
    ROS based platforms without the setup and configuration that ROSRV
    requries. This provides a tradeoff of greater usability with less
    guarantees with just ROSMOP, and significant setup time but much
    stronger guarantees with ROSRV.

The dL plugin is one of the latest additions to RV-Monitor’s Logic
Repository, and has been tested on simulations that were compiled to
LLVM. Recently, the team at Formal Systems Laboratory has focused on
isolating ROSMOP as a standalone tool, and enabling ROSMOP to use
RV-Monitor’s code generation capabilities.

In the following sections, we describe the building and testing
infrastructure we use to drive development of all three tools.

### Build and Testing Infrastructure

  - ROSMOP, originally an ant managed project, has been migrated over to
    Maven. This allows easier synchronization between the two projects
    as both use standard maven directory structures. RV-Monitor has been
    added as a dependency for ROSMOP. Unit tests are being added to
    ROSMOP to ensure greater stability.

  - Along with unit tests, we’ve developed a repository with
    specifications and code they must be run against. These
    specifications and corresponding test are designed to test the
    cohesiveness of all three codebases as a unit. For instance, for a
    given dL specficiation, we have as test LLVM and ROS simulations.
    Our testing setup generates monitors C++ and C code using ROSMOP and
    RV-Monitor respectively. The C++ code generated by ROSMOP includes,
    for as part of the monitoring logic C code generated by RV-Monitor’s
    dL plugin. The generated monitors are then used as follows -
    
    1)  For an LLVM simulation, RV-Monitor’s LLVM instrumentation is
        used to to instrument the simulation, and a monitored simulation
        run is compared against an expected run.
    
    2)  For ROSMOP, the generated C++ code is compiled as an independent
        monitoring node, and inserted into a running ROS test
        simulation.
    
    3)  For ROSRV, the setup creates a temporary catkin workspace, and
        compiles RVMaster with monitoring code produced by ROSMOP.
        RVMaster is then lauched, along with the simulation nodes, and
        the run is tested against an expected output.

Along with unit tests in each repository, we use the above as
integration tests, which allows us to move towards a stable build
infrastructure, as our codebase grows. Due to the nature of some of the
models we use as tests, our repository for integration tests at the
moment is not public.

## Contribution Guidelines

Contributions are welcome. Here are our expected guidelines. Our
guidelines are flexible, and will be flushed out in greater detail as we
make progress.

  - We’re moving towards a TDD model. Depending on the nature of
    contribution, we recommend the following -
    
    1)  If a contribution spans multiple tools, like a change in the way
        one of the main logic plugins work, we’d first encourage
        discussion with one of the maintainers before starting. The next
        step would be to come up with a specification or set of
        specifications and corresponding integration tests for said
        specifications. For a major change, it’s likely these
        integration tests will fail, which is ok. The idea is to clearly
        map out the purpose of the change. One of the repository
        maintainers can help get started with a collection utilities,
        such as bash scripts and Makefiles that manage ROS catkin
        workspaces efficiently without polluting your shell’s eviroment
        and test output. Have the repositories you’d like to make the
        change in as submodules within your “integration testing”
        environment. Once the initial specfication is mapped out, move
        inwards into the repository of interest, and add unit tests for
        smaller units of code that you plan to implement in order to
        make the change. You may have to modify existing test, depending
        on the situation. For ROSMOP and RV-Monitor, we mainly use JUnit
        along with AssertJ (see respectively pom.xml files for details).
        Move outwards towards as you add more unit tests, until
        eventually your integration tests start to pass.
    
    2)  For smaller changes, the process would roughly be the same,
        except you’d be working within the confines of a single
        repository. Add unit tests and create a PR. Our integration
        tests will ensure that your changes don’t break functionality
        across repositories.

  - For our development cycle, we rebase changes on top of master to
    avoid merge commits from diluting the semantics of master’s history.
    Thus, we recommend rebasing often and conflicts as they arise. The
    only tests that fail when you rebase, for every rebaed commit should
    be the integration or unit tests for your contribution that’s still
    incomplete. For smaller changes. When it comes to move your changes
    over to master, we’ll work with you to manage commits in a way such
    that each commit passes all tests. For commits themselves, we expect
    the filenames text that explains the change in the message itself.
    In essence, we’d like to be in a situation where on master, by
    examining the message, we get an idea of the change. In most cases,
    if a commit is too big to describe, it may be possible to break it
    up into smaller commits.

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
systems.” https://ls.cs.cmu.edu/KeYmaeraX. 

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

<div id="ref-mavenUrl">

\[11\] “The apache maven project.” https://maven.apache.org. 

</div>

<div id="ref-DL79">

\[12\] D. Harel, *First-order dynamic logic*. Berlin, Heidelberg:
Springer-Verlag, 1979. 

</div>

</div>

1.  <https://github.com/runtimeverification/rv-monitor/blob/master/plugins_logicrepository/dl/src/main/java/com/runtimeverification/rvmonitor/logicrepository/plugins/dl/DLPlugin.java>

2.  <https://github.com/runtimeverification/rv-monitor/blob/master/logicrepository/src/main/java/com/runtimeverification/rvmonitor/logicrepository/plugins/LogicPlugin.java>

3.  <http://wiki.ros.org/catkin/conceptual_overview>
