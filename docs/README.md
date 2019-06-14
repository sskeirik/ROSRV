# An Overview of MOP based Runtime Verification for ROS

## Introduction

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

## Preliminaries

### Monitoring Oriented Programming (MOP)

In essence MOP is a framework that uses runtime monitoring as a
fundamental principle for building reliable systems \[1\]. (Please see
<http://fsl.cs.illinois.edu/index.php/MOP_Papers> for a complete list of
paper on MOP).

## References

<div id="refs" class="references">

<div id="ref-MOP2012">

\[1\] G. Roşu and F. Chen, “Semantics and algorithms for parametric
monitoring,” *Logical Methods in Computer Science*, vol. 8, no. 1, pp.
1–47, 2012. 

</div>

</div>
