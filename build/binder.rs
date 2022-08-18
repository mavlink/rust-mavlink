use quote::format_ident;
use std::io::Write;

pub fn generate<W: Write>(modules: Vec<String>, out: &mut W) {
    let modules_tokens = modules.into_iter().map(|module| {
        let module_ident = format_ident!("{}", module);

        quote! {
            #[allow(non_camel_case_types)]
            #[allow(non_snake_case)]
            #[allow(unused_variables)]
            #[allow(unused_mut)]
            #[cfg(feature = #module)]
            pub mod #module_ident;
        }
    });

    let tokens = quote! {
        #(#modules_tokens)*
    };

    writeln!(out, "{}", tokens).unwrap();
}
