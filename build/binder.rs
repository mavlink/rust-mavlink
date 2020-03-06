use quote::Ident;
use std::io::Write;

pub fn generate<W: Write>(modules: Vec<String>, out: &mut W) {
    let modules_tokens = modules.into_iter().map(|module| {
        let file_name = module.clone() + ".rs";
        let module_ident = Ident::from(module.clone());

        let feature_gate = match module.as_str() {
            "common" => None,
            _ => Some(quote! { #[cfg(feature = #module)] }),
        };

        quote! {
            #[allow(non_camel_case_types)]
            #[allow(non_snake_case)]
            #[allow(unused_variables)]
            #[allow(unused_mut)]
            #feature_gate
            pub mod #module_ident {
                use crate::MavlinkVersion; //TODO verify
                include!(#file_name);
            }
        }
    });

    let tokens = quote! {
        #(#modules_tokens)*
    };

    writeln!(out, "{}", tokens).unwrap();
}
