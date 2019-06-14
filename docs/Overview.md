An Overivew MOP based runtime verification for ROS
==================================================

Introduction
------------

The purpose of this document is to -
  a) Provide a brief theoretical background behind three
     distinct projects that are used in unison for Runtime Verification (RV) of
     ROS based application -
      * [ROSRV](https://github.com/Formal-Systems-Laboratory/ROSRV/),
      * [RV-Monitor](https://github.com/runtimeverification/rv-monitor) and
      * [ROSMOP](https://github.com/Formal-Systems-Laboratory/rosmop)
  b) Discuss the *overall architecture* needed for RV of ROS applications using aforementioned tools.
     The tools are maintained in separate repositories for modularity. Each tool
     can be used independently, but most use cases require tight interaction. We
     describe our current development and testing setup that allows us the advantages
     of having a "mono-repo" like setup.
  c) Layout guidelines for -
     * Contributions to each of the codebase, or to the fundamental theory
       behind them.
     * Setting up appropriate RV infrastructure based on need.
  d) Describe the current status of development, immediate goals, and a roadmap
     for long term goals.
