#import "@preview/glossarium:0.5.4": *
#import "@preview/linguify:0.4.2": *
#import "@preview/glossarium:0.5.9": make-glossary, register-glossary, print-glossary, gls, glspl
#import "@preview/glossarium:0.5.4": *
#import "dataflow.typ": dataflow
#import "preesm.typ": preesm 
#import "radioastro.typ": radioastro 
#show: make-glossary

#show link: underline
#set text(lang: "fr")

#set page(margin: (
  top: 1cm,
  bottom: 1cm,
  x: 1cm,
))



= PREESM
#preesm 


= DATAFLOW
#dataflow 


= RADIOASTRO 
#radioastro


#bibliography("biblio.bib", full: true)