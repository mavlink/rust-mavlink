use quote::{format_ident, quote};
use std::io::Write;

pub fn generate<W: Write>(modules: Vec<String>, out: &mut W) {
    let modules_tokens = modules.into_iter().map(|module| {
        let module_ident = format_ident!("{}", module);

        quote! {
            #[allow(non_camel_case_types)]
            #[allow(clippy::derive_partial_eq_without_eq)]
            #[allow(clippy::field_reassign_with_default)]
            #[allow(non_snake_case)]
            #[allow(clippy::unnecessary_cast)]
            #[allow(clippy::bad_bit_mask)]
            #[cfg(feature = #module)]
            pub mod #module_ident;
        }
    });

    let tokens = quote! {
        #(#modules_tokens)*
    };

    writeln!(out, "{tokens}").unwrap();
}
