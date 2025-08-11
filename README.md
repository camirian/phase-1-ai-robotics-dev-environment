# Phase 1: AI Robotics Professional Development Environment

This repository documents the complete setup and configuration of a professional-grade development environment for advanced robotics and AI applications. The goal is to establish a robust, reproducible, and distributed system consisting of a powerful host development workstation and a physical edge AI device (NVIDIA Jetson).

This setup serves as the foundational environment for all subsequent projects in my AI Robotics Portfolio.

---

## 🚀 Key Components

1.  **Host Development Workstation:** A powerful PC running Ubuntu 22.04, equipped with an NVIDIA RTX GPU for high-fidelity simulation and AI model training.
2.  **Edge AI Device:** An NVIDIA Jetson Orin Nano for developing and testing "sim-to-real" deployments, running perception and control algorithms on physical hardware.

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

-   **[`host-pc-setup.md`](./host-pc-setup.md):** Complete guide for setting up the host workstation, from OS installation to NVIDIA drivers.
-   **[`jetson-setup.md`](./jetson-setup.md):** Complete guide for flashing and configuring the Jetson Orin Nano and verifying network communication.

---

## ✅ Status

**Phase 1 is 100% complete and verified.** The host and edge devices are fully configured and can communicate seamlessly over the network using ROS 2.