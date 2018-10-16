//! Ensemble des structures de représentation des objets électroniques
//!
//! Le mod `c_struct` permets d'avoir une représentation bas niveau des modules
//! dont l'échange de données avec l'informatique se fait par des fonctions C

mod c_struct;
pub mod servos;
pub use structs::servos::Servos2019;

#[cfg(test)]
mod tests;
