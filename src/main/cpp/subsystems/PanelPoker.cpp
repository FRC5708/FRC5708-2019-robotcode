#include "subsystems/PanelPoker.h"

PanelPoker::PanelPoker() : Subsystem("Panel Poker"), 
pokerSolenoid(new frc::DoubleSolenoid(pokerSolenoidChannelForward, pokerSolenoidChannelBackward)){
    
}

void PanelPoker::Deploy(){
    pokerSolenoid->Set(pokerSolenoid->kForward);
}

void PanelPoker::Stow(){
    pokerSolenoid->Set(pokerSolenoid->kReverse);
}