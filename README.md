# Phase 1: AI Robotics Professional Development Environment

This repository documents the complete setup and configuration of a professional-grade development environment for advanced robotics and AI applications. The goal is to establish a robust, reproducible, and distributed system consisting of a powerful host development workstation and a physical edge AI device (NVIDIA Jetson).

This setup serves as the foundational environment for all subsequent projects in my AI Robotics Portfolio.

---

## ✅ Skills Demonstrated

Successfully completing this foundational setup demonstrates core competencies in:

-   **Systems Administration:** Proficiently installing and configuring a Linux (Ubuntu 22.04) environment from scratch, including disk partitioning for dual-boot systems.
-   **Hardware & Driver Management:** Correctly installing and verifying proprietary NVIDIA drivers on Linux, including handling dependencies and Secure Boot (MOK) enrollment.
-   **Distributed Systems & Networking:** Establishing and verifying a multi-machine ROS 2 network, proving an understanding of distributed computing and the DDS discovery mechanism.
-   **Embedded & Edge AI Systems:** Flashing and configuring an embedded device (NVIDIA Jetson) with the JetPack SDK, preparing it for sim-to-real deployment.
-   **Version Control & Professional Documentation:** Utilizing Git and GitHub for version control and maintaining high-quality, structured documentation for a technical project.

---

## 🚀 Key Components

1.  **Host Development Workstation:** A powerful PC running Ubuntu 22.04, equipped with an NVIDIA RTX GPU for high-fidelity simulation and AI model training.
2.  **Edge AI Device:** An NVIDIA Jetson Orin Nano for developing and testing "sim-to-real" deployments.

---

## 🛠️ Software Stack

| Component             | Version / Type                   | Purpose                                    |
| --------------------- | -------------------------------- | ------------------------------------------ |
| Operating System      | Ubuntu 22.04 LTS                 | Standard for robotics development          |
| Robotics Middleware   | ROS 2 Humble Hawksbill           | Core communication and tooling framework   |
| GPU Driver            | NVIDIA Proprietary Driver 5xx.xx | Enables GPU acceleration for AI/Sim        |
| Simulation Platform   | NVIDIA Isaac Sim                 | High-fidelity physics simulation & sensor data |
| Edge AI SDK           | NVIDIA JetPack                   | OS & libraries for the Jetson platform     |

---

## 📝 Setup & Verification Details

Detailed, step-by-step instructions and verification checklists for each component are located in the following documents:

-   [`host-pc-setup.md`](./host-pc-setup.md): Complete guide for the host workstation setup.
-   [`jetson-setup.md`](./jetson-setup.md): Complete guide for the Jetson Orin Nano setup.

---

## 📜 License

This project is licensed under the Apache 2.0 License. See the [`LICENSE`](./LICENSE) file for details.