//! Ce module contiens des fonctions utilitaires.

/// Cette fonction prends en entrée une `Slice` et renvoie un tuple contenant :
/// * un tableau contenant les 8 premiers éléments de `slice`.
/// * le nombre d'élements de slice qui ont été copiés
pub fn slice_to_array_8<T>(slice: &[T]) -> ([T; 8], u8)
where
    T: Default + Copy,
{
    let mut array: [T; 8] = Default::default();
    let mut count: u8 = 0;
    for (&x, p) in slice.iter().zip(array.iter_mut()) {
        *p = x;
        count += 1;
    }
    (array, count)
}
