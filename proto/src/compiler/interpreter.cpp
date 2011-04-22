/* Proto interpreter
Copyright (C) 2009, Jacob Beal, and contributors 
listed in the AUTHORS file in the MIT Proto distribution's top directory.

This file is part of MIT Proto, and is distributed under the terms of
the GNU General Public License, with a linking exception, as described
in the file LICENSE in the MIT Proto distribution's top directory. */

// The Proto interpreter turns S-Expressions into intermediate representation

#include "config.h"
#include "nicenames.h"
#include "compiler.h"

/*****************************************************************************
 *  DUMMY ELEMENTS                                                           *
 *****************************************************************************/
// Dummies are used to fill in gaps created by syntax errors, allowing the
// compiler to find multiple errors in one pass & exit gracefully

CompilationElement* dummy(string type, CompilationElement* context) {
  CompilationElement *elt;
  if(type=="Operator") elt = Env::core_op("tup"); // takes anything
  else if(type=="Type") elt = new ProtoType();
  else if(type=="Signature") elt = new Signature(context);
  else if(type=="Macro") elt = new Macro("*ERROR*",new SE_Symbol("*ERROR*"));
  else if(type=="SExpr") elt = new SE_Symbol("*ERROR*");
  else ierror("Don't know how to make dummy value for "+type);
  elt->inherit_attributes(context); // in case not already marked
  elt->attributes["DUMMY"]=new MarkerAttribute(true);
  return elt;
}

Field* field_err(CompilationElement *where,AM* space,string msg) {
  compile_error(where,msg); 
  OI* oi = new OI(where,(Operator*)dummy("Operator",where),space);
  oi->attributes["DUMMY"]=new MarkerAttribute(true);
  return oi->output;
}
Operator* op_err(CompilationElement *where,string msg) {
  compile_error(where,msg); return (Operator*)dummy("Operator",where);
}
Macro* macro_err(CompilationElement *where,string msg) {
  compile_error(where,msg); return (Macro*)dummy("Macro",where);
}
ProtoType* type_err(CompilationElement *where,string msg) {
  compile_error(where,msg); return (ProtoType*)dummy("Type",where);
}
Signature* sig_err(CompilationElement *where,string msg) {
  compile_error(where,msg); return (Signature*)dummy("Signature",where);
}
SExpr* sexp_err(CompilationElement *where,string msg) {
  compile_error(where,msg); return (SExpr*)dummy("SExpr",where);
}


/*****************************************************************************
 *  ENVIRONMENTS                                                             *
 *****************************************************************************/

// specials are the tokens that are hard-wired in and thus can't be shadowed
set<string> special_tokens;
bool specials_populated = false;
void populate_specials() {
  if(specials_populated) return;
  special_tokens.insert("lambda"); special_tokens.insert("fun");
  special_tokens.insert("def"); special_tokens.insert("macro");
  special_tokens.insert("primitive"); special_tokens.insert("annotate");
  special_tokens.insert("let"); special_tokens.insert("let*"); 
  special_tokens.insert("letfed"); special_tokens.insert("letfed+");
  special_tokens.insert("restrict");
  special_tokens.insert("all");
  special_tokens.insert("include"); special_tokens.insert("tup");
  special_tokens.insert("true"); special_tokens.insert("false");
  specials_populated=true;
}
bool is_special(SExpr* s) {
  if(!s->isSymbol()) return false;
  return special_tokens.count(((SE_Symbol*)s)->name);
}

// tokens that need to be always available to the interpreter, for 
// expressions that have a syntactic component
void Env::bind(string name, CompilationElement* value) {
  if(bindings.count(name)) 
    compile_error(value,"Cannot bind '"+name+"': already bound");
  force_bind(name,value);
}

void Env::force_bind(string name, CompilationElement* value) {
  if(special_tokens.count(name)) 
    compile_error(value,"Cannot bind '"+name+"': symbol is reserved");
  else if(core_ops.count(name))
    compile_warn(value,"shadowing core operator '"+name+"'");
  bindings[name]=value;
}

CompilationElement* Env::lookup(SE_Symbol* sym, string type) {
  CompilationElement* found = lookup(sym->name);
  if(found) {
    if(found->isA(type)) return found;
    else compile_error(sym,sym->name+" is "+found->type_of()+", not "+type);
  } else compile_error(sym,"Couldn't find definition of "+type+" "+sym->name);
  return dummy(type,sym);
}

CompilationElement* Env::lookup(string name, bool recursed) {
  if(bindings.count(name)) { return bindings[name]; // local search
  } else if(parent) { return parent->lookup(name); // search through parents
  } else if(!recursed) { // check for a file to define it
    string fname = name + ".proto";
    if(cp->parent->proto_path.find_in_path(fname)) {
      cp->interpret_file(fname);
      return lookup(name,true);
    }
  }
 return NULL;
}

// Access to operators that the compiler must be able to get at unshadowed
map<string,Operator*> Env::core_ops;
void Env::record_core_ops(Env* toplevel) {
  map<string,CompilationElement*>::iterator i;
  for(i=toplevel->bindings.begin();i!=toplevel->bindings.end();i++)
    if(i->second->isA("Operator")) core_ops[i->first]=(Operator*)i->second;
}
Operator* Env::core_op(string name) {
  if(!core_ops.count(name)) ierror("Compiler missing core operator '"+name+"'");
  return core_ops[name];
}


/*****************************************************************************
 *  TYPES                                                                    *
 *****************************************************************************/

ProtoType* ProtoInterpreter::sexp_to_type(SExpr* s) {
  if(s->isSymbol()) {
    string name = ((SE_Symbol*)s)->name;
    if(name=="any") { return new ProtoType();
    } else if(name=="local") { return new ProtoLocal();
    } else if(name=="tuple") { return new ProtoTuple();
    } else if(name=="symbol") { return new ProtoSymbol();
    } else if(name=="number") { return new ProtoNumber();
    } else if(name=="scalar") { return new ProtoScalar();
    } else if(name=="boolean") { return new ProtoBoolean();
    } else if(name=="vector") { return new ProtoVector();
    } else if(name=="lambda" || name=="fun") { return new ProtoLambda();
    } else if(name=="field") { return new ProtoField();
    } else { return type_err(s,"Unknown type "+s->to_str());
    }
  } else if(s->isList()) {
    SE_List* sl = (SE_List*)s;
    if(!sl->op()->isSymbol()) 
      return type_err(s,"Compound type must start with symbol: "+ce2s(s));
    string name = ((SE_Symbol*)sl->op())->name;
    if(name=="tuple" || name=="vector") {
      ProtoTuple* t;
      if(name=="tuple") t=new ProtoTuple(true); else t=new ProtoVector(true);
      for(int i=1;i<sl->len();i++) {
        SExpr* subex = (*sl)[i];
        if(subex->isSymbol() && ((SE_Symbol*)subex)->name=="&rest") {
          t->bounded=false; continue;
        }
        ProtoType* sub = sexp_to_type(subex);
        if(name=="vector" && !sub->isA("ProtoScalar"))
          return type_err(sl,"Vectors must contain only scalars");
        t->types.push_back(sub);
      }
      return t;
    } else if(name=="lambda" || name=="fun") {
      if(sl->len()!=3) return type_err(s,"Bad lambda type: "+s->to_str());
      Signature* sig = sexp_to_sig((*sl)[1]);
      sig->output = sexp_to_type((*sl)[2]);
      return new ProtoLambda(new Operator(s,sig));
    } else if(name=="field") {
      if(sl->len()!=2) return type_err(s,"Bad field type: "+s->to_str());
      ProtoType* sub = sexp_to_type((*sl)[1]);
      if(sub->isA("ProtoField")) 
        return type_err(s,"Field type must have a local subtype");
      return new ProtoField(sub);
    } else {
      return type_err(s,"Unknown type "+s->to_str());
    }
  } else { // scalars specify ProtoScalar literals
    return new ProtoScalar(((SE_Scalar*)s)->value);
  }
}

// test whether expression is a type expression (though not necessarily
// correctly formatted) without actually trying to interpret
bool ProtoInterpreter::sexp_is_type(SExpr* s) {
  if(s->isSymbol()) {
    string name = ((SE_Symbol*)s)->name;
    if(name=="any") return true;
    if(name=="local") return true;
    if(name=="tuple") return true;
    if(name=="symbol") return true;
    if(name=="number") return true;
    if(name=="scalar") return true;
    if(name=="boolean") return true;
    if(name=="vector") return true;
    if(name=="lambda" || name=="fun") return true;
    if(name=="field") return true;
  } else if(s->isList()) {
    SE_List* sl = (SE_List*)s;
    if(!sl->op()->isSymbol()) return false;
    string name = ((SE_Symbol*)sl->op())->name;
    if(name=="tuple" || name=="vector") return true;
    if(name=="lambda" || name=="fun") return true;
    if(name=="field") return true;
  } else { // scalars specify ProtoScalar literals
    return true;
  }
  return false;
}

/*****************************************************************************
 *  MACROS                                                                   *
 *****************************************************************************/

// gensyms give guaranteed unique names, used to prevent variable capture
int MacroOperator::gensym_count=0;
bool is_gensym(SE_Symbol* s) { return s->name[0]=='?'; }
bool is_gensym(SExpr* s) { return s->isSymbol() && is_gensym((SE_Symbol*)s); }
SE_Symbol* make_gensym(string root) {
  size_t breakloc = root.rfind("~"); // try to strip previous gensym #s
  if(breakloc!=string::npos && str_is_number(root.substr(breakloc+1).c_str()))
    root = root.substr(0,breakloc);
  return new SE_Symbol(root+"~"+int2str(MacroOperator::gensym_count++));
}

// Macro signatures are simpler because they are syntactic, not semantic
Signature* ProtoInterpreter::sexp_to_macro_sig(SExpr* s) {
  if(!s->isList()) return sig_err(s,"Signature not a list: "+s->to_str());
  SE_List* sl = (SE_List*)s;  Signature* sig = new Signature(s);
  int stage = 0; // 0=required, 1=optional, 2=rest
  vector<SExpr*>::iterator it;
  for(it=sl->children.begin(); it<sl->children.end(); it++) {
    if(!(*it)->isSymbol()) 
      return sig_err(s,"Bad signature structure: "+s->to_str());
    string name = ((SE_Symbol*)*it)->name;
    if(name=="&optional") {
      if(stage>0) return sig_err(s,"Bad signature structure: "+s->to_str());
      stage=1; continue;
    } else if(name=="&rest") {
      if(stage>1) return sig_err(s,"Bad signature structure: "+s->to_str());
      stage=2; continue;
    } 
    // non-special symbols fall through to here
    ProtoType* type = new ProtoSymbol(name);
    switch(stage) {
    case 0: sig->required_inputs.push_back(type); break;
    case 1: sig->optional_inputs.push_back(type); break;
    case 2: sig->rest_input = type; stage=3; break; // only one rest
    case 3: return sig_err(s,"Bad signature structure: "+s->to_str());
    default: ierror("Unknown stage parsing macro signature: "+s->to_str());
    }
  }
  return sig;
}

Macro* ProtoInterpreter::sexp_to_macro(SE_List* s, Env *env) {
  if(!(*s)[1]->isSymbol()) 
    return macro_err(s,"Bad macro name: "+(*s)[1]->to_str());
  string name = ((SE_Symbol*)(*s)[1])->name;
  Macro* m;
  if(s->len()==3) { // symbol-style macro
    if(is_special((*s)[2])) 
      return macro_err(s,"Cannot rename specials: "+(*s)[1]->to_str());
    m = new MacroSymbol(name,(*s)[2]);
  } else if(s->len()==4) { // operator-style macro
    Signature *sig = sexp_to_macro_sig((*s)[2]);
    m = new MacroOperator(name,sig,(*s)[3]);
    env->force_bind(name,m); return m; // bind & return
  } else {
    return macro_err(s,"Macro must have 3 or 4 arguments: "+s->to_str());
  }
  env->force_bind(name,m); return m; // bind & return
}

SExpr* ProtoInterpreter::expand_macro(MacroOperator* m, SE_List* call) {
  Env m_env(this);
  // bind variables to SExprs
  if(!m->signature->legal_length(call->len()-1))
    return sexp_err(call,"Wrong number of arguments for macro "+m->name);
  int i=1; // start after macro name
  for(int j=0;j<m->signature->required_inputs.size();j++) {
    ProtoSymbol* var = dynamic_cast<ProtoSymbol*>(m->signature->required_inputs[j]);
    m_env.bind(var->value,(*call)[i++]);
  }
  for(int j=0;j<m->signature->optional_inputs.size() && i<call->len();j++) {
    ProtoSymbol* var = dynamic_cast<ProtoSymbol*>(m->signature->optional_inputs[j]);
    m_env.bind(var->value,(*call)[i++]);
  }
  if(i<call->len()) { // sweep all else into rest argument
    ProtoSymbol* var = dynamic_cast<ProtoSymbol*>(m->signature->rest_input);
    SE_List *rest = new SE_List(); rest->inherit_attributes(m);
    for(; i<call->len(); ) { rest->add((*call)[i++]); }
    m_env.bind(var->value,rest);
  }
  // then substitute the pattern
  return macro_substitute(m->pattern,&m_env);
}

// walks through, copying (and inheriting attributes)
SExpr* ProtoInterpreter::macro_substitute(SExpr* src, Env* e, SE_List* wrapper) {
  if(is_gensym(src)) { // substitute with a gensym for this instance
    SE_Symbol *groot = (SE_Symbol*)src;
    SExpr* gensym = (SExpr*)e->lookup(groot->name);
    if(!gensym) { // if gensym not yet created, make & bind it
      gensym = make_gensym(groot->name); gensym->inherit_attributes(src);
      e->bind(groot->name,gensym);
    }
    return gensym;
  } else if(src->isList()) { // SE_List
    SE_List *srcl = (SE_List*)src;
    string opname = (*srcl)[0]->isSymbol() ? ((SE_Symbol*)(*srcl)[0])->name :"";
    if(opname=="comma") {
      if(srcl->len()!=2 || !(*srcl)[1]->isSymbol())
        return sexp_err(src,"Bad comma form: "+src->to_str());
      SE_Symbol* sn = (SE_Symbol*)(*srcl)[1];
      return ((SExpr*)e->lookup(sn,"SExpr"))->copy(); // insert source text
    } else if(opname=="comma-splice") {
      if(wrapper==NULL)
        return sexp_err(src,"Comma-splice "+(*srcl)[0]->to_str()+" w/o list");
      if(srcl->len()!=2 || !(*srcl)[1]->isSymbol())
        return sexp_err(src,"Bad comma form: "+src->to_str());
      SE_Symbol* sn = (SE_Symbol*)(*srcl)[1];
      SE_List* value = (SE_List*)e->lookup(sn,"SE_List");
      for(int i=0;i<value->len();i++) 
        wrapper->add((*value)[i]->copy());
      return NULL; // comma-splices return null
    } else { // otherwise, just substitute each child
      SE_List *l = new SE_List(), *srcl = (SE_List*)src;
      for(int i=0;i<srcl->len();i++) {
        SExpr* sub = macro_substitute((*srcl)[i],e,l);
        if(sub!=NULL) l->add(sub); // comma-splices add selves and return null
      }
      return l;
    }
  } else { // symbol or scalar
    return src->copy();
  }
}

/*****************************************************************************
 *  INTERPRETER CORE                                                         *
 *****************************************************************************/

// Parsing nth argument (output = -1)
pair<string,ProtoType*> ProtoInterpreter::parse_argument(SE_List_iter* i, int n,
                                                         Signature* sig, bool anonymous_ok) {
  SExpr *a = i->get_next("argument");
  SExpr *b = (i->on_token("|")) ? i->get_next("argument") : NULL;
  string name=""; ProtoType* type=NULL;
  if(b) { // specified as name|type
    if(sexp_is_type(a))
      compile_error(a,"Parameter name cannot be a type");
    if(a->isSymbol()) name = ((SE_Symbol*)a)->name;
    else compile_error(a,"Parameter name not a symbol: "+ce2s(a));
    type = sexp_to_type(b);
  } else { // determine name or type by parsing
    if(sexp_is_type(a)) {
      if(anonymous_ok) type = sexp_to_type(a);
      else compile_error(a,"Function parameters must be named: "+ce2s(a));
    }
    else if(a->isSymbol()) name = ((SE_Symbol*)a)->name;
    else compile_error(a,"Parameter name not a symbol: "+ce2s(a));
  }
  // fall back to defaults where needed
  if(name=="") name = (n>=0) ? ("arg"+i2s(n)) : "value";
  if(type==NULL) type = new ProtoType();
  // record name in signature and return
  if(sig->names.count(name))
    compile_error(a,"Cannot bind '"+name+"': already bound");
  sig->names[name] = n;
  return make_pair(name,type);
}

// if bindloc = NULL, then it's a primitive and expressions are types
// otherwise, it's a normal signature, and they're variables to be bound
Signature* ProtoInterpreter::sexp_to_sig(SExpr* s, Env* bindloc, CompoundOp* op, AM* space){
  if(!s->isList()) return sig_err(s,"Signature not a list: "+s->to_str());
  Signature* sig = new Signature(s);
  int stage = 0; // 0=required, 1=optional, 2=rest
  int varid = -1; // index to current parameter
  SE_List_iter li((SE_List*)s);
  while(li.has_next()) {
    if(li.on_token("&optional")) {
      if(stage>0) return sig_err(s,"Misplaced signature '&optional': "+ce2s(s));
      stage=1; continue;
    } else if(li.on_token("&rest")) {
      if(stage>1) return sig_err(s,"Misplaced signature '&rest': "+ce2s(s));
      stage=2; continue;
    }
    // Otherwise, it's an entry in the signature: name, type, or name|type
    pair<string,ProtoType*> arg = parse_argument(&li,++varid,sig,bindloc==NULL);
    // Create parameter for functions:
    if(bindloc) {
      Parameter *p = new Parameter(op,arg.first,varid,arg.second);
      bindloc->bind(arg.first,(new OI(s,p,space))->output);
    }
    switch(stage) {
    case 0: sig->required_inputs.push_back(arg.second); break;
    case 1: sig->optional_inputs.push_back(arg.second); break;
    case 2: sig->rest_input = arg.second; stage=3; break; // only one rest
    case 3: return sig_err(s,"No signature past '&rest' variable: "+ce2s(s));
    default: ierror("Unknown stage while parsing signature: "+ce2s(s));
    }
  }
  return sig;
}

// Parse the extra arguments of a primitive into attributes
void parse_primitive_attributes(SE_List_iter* li,Primitive* p) {
  while(li->has_next()) {
    SExpr* v = li->get_next();
    if(!v->isKeyword()) {compile_error(v,v->to_str()+" not a keyword"); return;}
    if(p->attributes.count(((SE_Symbol*)v)->name))
      compile_warn("Primitive "+p->name+" overriding duplicate '"
                   +((SE_Symbol*)v)->name+"' attribute");
    if(li->has_next() && !li->peek_next()->isKeyword()) {
      p->attributes[((SE_Symbol*)v)->name]=new SExprAttribute(li->get_next());
    } else {
      p->attributes[((SE_Symbol*)v)->name]=new MarkerAttribute(true);
    }
  }
}

Operator* ProtoInterpreter::sexp_to_op(SExpr* s, Env *env) {
  if(s->isSymbol()) {
    return (Operator*)env->lookup((SE_Symbol*)s,"Operator");
  } else if(s->isScalar()) {return op_err(s,s->to_str()+" is not an Operator");
  } else { // it must be a list
    SE_List_iter li(s);
    string opdef = li.get_token("operator type");
    // (def name sig body) - constructs operator & graph, then binds
    // (lambda sig body) - constructs operator & graph
    if(opdef=="def" || opdef=="lambda" || opdef=="fun") {
      int i=1;
      string name;
      if(opdef=="def") name=li.get_token("function name");
      // setup the operator & its signature
      CompoundOp* op = new CompoundOp(s,dfg,name);
      if(opdef=="def") env->force_bind(name,op);// bind early to allow recursion
      Env* inner = new Env(env);
      op->signature = sexp_to_sig(li.get_next("signature"),inner,op,op->body);
      op->signature->output = new ProtoType();
      V4 << "Creating function "<<op->name<<", body: "<<ce2s(op->body)<<endl;
      // make body sexpr
      SE_List bodylist; bodylist.add(new SE_Symbol("all"));
      while(li.has_next()) bodylist.add(li.get_next());
      SExpr* body = (bodylist.len()==2)?bodylist.children[1]:&bodylist;
      // parsing body sexpr & collect real output value
      op->output = sexp_to_graph(body,op->body,inner);
      if(op->output==NULL) op->output=field_err(s,op->body,"Function has no content");
      op->compute_side_effects();
      return op;
    } else if(opdef=="primitive") { // constructs & binds operator w/o DFG
      // (primitive name sig out &optional :side-effect)
      string pname = li.get_token("primitive name");
      V4 << "Creating new primitive "<<pname<<endl;
      Signature* sig = sexp_to_sig(li.get_next());
      sig->output = parse_argument(&li,-1,sig).second;
      Operator* p  = new Primitive(s,pname,sig);
      parse_primitive_attributes(&li,(Primitive*)p); // add in attributes
      env->force_bind(pname,p); return p;
    } else {
      // check if it's a macro
      CompilationElement* ce = env->lookup(opdef);
      if(ce && ce->isA("Macro")) {
        V4 << "Expanding macro "<<ce2s(ce)<<endl;
        SExpr* new_expr;
        if(ce->isA("MacroOperator")) {
          new_expr = expand_macro((MacroOperator*)ce,(SE_List*)s);
          if(new_expr->attributes.count("DUMMY")) // Mark of a failure
            return op_err(s,"Macro expansion failed on "+s->to_str());
        } else { // it's a MacroSymbol
          new_expr = s->copy();
          ((SE_List*)new_expr)->children[0]=((Macro*)ce)->pattern;
        }
        return sexp_to_op(new_expr,env);
      }
    }
    return op_err(s,"Can't make an operator with "+opdef);
  }
  ierror("Fell through sexp_to_op w/o returning for: "+s->to_str());
}

Field* ProtoInterpreter::let_to_graph(SE_List* s, AM* space, Env *env, 
                                 bool incremental) { // incremental -> let*
  if(s->len()<3 || !s->children[1]->isList())
    return field_err(s,space,"Malformed let statement: "+s->to_str());
  Env* child = new Env(env);
  vector<SExpr*>::iterator let_exps = s->args();
  // collect let declarations
  SE_List* decls = (SE_List*)*let_exps++;
  for(int i=0;i<decls->len();i++) {
    if((*decls)[i]->isList()) {
      SE_List* d = (SE_List*)(*decls)[i];
      if(d->len()==2 && (*d)[0]->isSymbol()) {
        V4 << "Creating let variable "<<ce2s((*d)[0])<<endl;
        Field* f = sexp_to_graph((*d)[1],space,(incremental?child:env));
        child->bind(((SE_Symbol*)(*d)[0])->name,f);
      } else compile_error(d,"Malformed let statement: "+d->to_str());
    } else compile_error((*decls)[i],"Malformed let statement: "+
                         (*decls)[i]->to_str());
  }
  // evaluate body in child environment, returning last output
  Field* out=NULL;
  for(; let_exps<s->children.end(); let_exps++)
    out = sexp_to_graph(*let_exps,space,child);
  return out;
}

// A tuple-style letfed has the following behavior:
// 1. its MUX outputs a tuple
// 2. it decomposes said tuple to bind children
// returns true if binding's OK
bool bind_letfed_vars(SExpr* s, Field* val, AM* space, Env* env) {
  bool ok=false;
  if(s->isSymbol()) { // base case: just bind the variable
    env->bind(((SE_Symbol*)s)->name,val); ok=true;
  } else if(s->isList()) { // tuple variable?
    SE_List *sl = (SE_List*)s;
    if(sl->op()->isSymbol() && ((SE_Symbol*)sl->op())->name=="tup") {
      ok=true;
      for(int i=1;i<sl->len();i++) {
        // make element accessor
        DFG* dfg = space->container;
        OI* accessor = new OperatorInstance(s,Env::core_op("elt"),space);
        accessor->add_input(val);
        accessor->add_input(dfg->add_literal(new ProtoScalar(i-1),space,s));
        // recurse binding process
        ok &= bind_letfed_vars((*sl)[i],accessor->output,space,env);
      }
    }
  } // unhandled = not OK
  return ok; 
}

Field* ProtoInterpreter::letfed_to_graph(SE_List* s, AM* space, Env *env,
                                         bool no_init) {
  if(s->len()<3 || !s->children[1]->isList())
    return field_err(s,space,"Malformed letfed statement: "+s->to_str());
  Env *child = new Env(env), *delayed = new Env(env);
  // unless no_init, make the two forks: init and update
  OI *dchange; AM *init, *update;
  if(!no_init) {
    dchange = new OperatorInstance(s,Env::core_op("dchange"),space);
    init = new AM(s,space,dchange->output);
    OI *unchange = new OperatorInstance(s,Env::core_op("not"),space);
    unchange->add_input(dchange->output);
    update = new AM(s,space,unchange->output);
    V4 << "Created feedback spaces:\n";
    V4 <<"- Init: "<<ce2s(init) <<" Update: "<<ce2s(update)<<endl;
  } else {
    dchange = NULL;//new OI(s,new Literal(s,new ProtoBoolean(false)),space);
    init = update = space;
  }
  // collect & bind let declarations, verify syntax
  vector<SExpr*>::iterator let_exps = s->args();
  SE_List* decls = (SE_List*)*let_exps++;
  vector<OperatorInstance*> vars;
  for(int i=0;i<decls->len();i++) {
    if((*decls)[i]->isList() && ((SE_List*)(*decls)[i])->len()==3) {
      SE_List* d = (SE_List*)(*decls)[i];
      V4 << "Creating feedback variable "<<ce2s((*d)[0])<<endl;
      // create the variable & bind it
      OI *varmux, *delay;
      if(!no_init) {
        varmux = new OperatorInstance(d,Env::core_op("mux"),space);
        varmux->attributes["LETFED-MUX"] = new MarkerAttribute(true);
        varmux->add_input(dchange->output);
        // create the delayed version
        delay = new OperatorInstance(d,Env::core_op("delay"),update);
        delay->add_input(varmux->output);
      } else {
        delay = varmux = new OperatorInstance(d,Env::core_op("delay"),update);
      }
      vars.push_back(varmux);
      // bind the variables
      if(!bind_letfed_vars(d->op(),delay->output,update,delayed))
        compile_error(d,"Malformed letfed variable: "+d->op()->to_str());
      // evaluate & connect the init
      if(!no_init) {
        varmux->add_input(sexp_to_graph((*d)[1],init,env));
      } else {
        delay->output->range = sexp_to_type((*d)[1]);
        //varmux->add_input(dfg->add_literal(new ProtoBoolean(false),space,s));
      }
    } else compile_error((*decls)[i],"Malformed letfed statement: "+
                         (*decls)[i]->to_str());
  }
  // second pass to evalute update expressions
  for(int i=0;i<decls->len();i++) {
    SE_List* d = (SE_List*)(*decls)[i];
    Field* up_value = sexp_to_graph((*d)[2],update,delayed);
    if(!no_init) {
      bind_letfed_vars(d->op(),vars[i]->output,space,child);
      vars[i]->add_input(up_value);
    } else {
      bind_letfed_vars(d->op(),up_value,space,child);
      vars[i]->add_input(up_value);
    }
  }
  // evaluate body in child environment, returning last output
  Field* out=NULL;
  for(; let_exps<s->children.end(); let_exps++)
    out = sexp_to_graph(*let_exps,space,child);
  return out;
}

Field* ProtoInterpreter::restrict_to_graph(SE_List* s, AM* space, Env *env){
  if(s->len()!=3) 
    return field_err(s,space,"Malformed restrict statement: "+s->to_str());
  AM* child = new AM(s,space,sexp_to_graph((*s)[2],space,env));
  V4 << "Restricting interpretation to " << ce2s(child) << endl;
  return sexp_to_graph((*s)[1],child,env);
}

// Returns an instance of the literal if it exists, or else NULL
ProtoType* ProtoInterpreter::symbolic_literal(string name) {
  if(name=="true") { return new ProtoBoolean(true);
  } else if(name=="false") { return new ProtoBoolean(false);
  } else return NULL;
}

ProtoLocal* quote_to_literal_type(SExpr* s) {
  if(s->isSymbol()) {
    return new ProtoSymbol(((SE_Symbol*)s)->name);
  } else if(s->isScalar()) {
    return new ProtoScalar(((SE_Scalar*)s)->value);
  } else { // SE_List
    SE_List* sl = (SE_List*)s;
    vector<ProtoType*> subs; bool all_scalar=true;
    for(int i=0;i<sl->len();i++) {
      ProtoType* sub = quote_to_literal_type((*sl)[i]); subs.push_back(sub);
      if(!sub->isA("ProtoScalar")) all_scalar=false;
    }
    ProtoTuple* out = all_scalar ? new ProtoVector(true) : new ProtoTuple(true);
    for(int i=0;i<subs.size();i++) out->add(subs[i]);
    return out;
  }
}

// returns the output field
Field* ProtoInterpreter::sexp_to_graph(SExpr* s, AM* space, Env *env) {
  V3 << "Interpret: " << ce2s(s) << " in " << ce2s(space) << endl;
  if(s->isSymbol()) {
    // All other symbols are looked up in the environment
    CompilationElement* elt = env->lookup(((SE_Symbol*)s)->name);
    if(elt==NULL) { 
      V4 << "Symbolic literal?\n";
      ProtoType* val = symbolic_literal(((SE_Symbol*)s)->name);
      if(val) { V4 << "- Yes\n"; return dfg->add_literal(val,space,s); }
      return field_err(s,space,"Couldn't find definition of "+s->to_str());
    } else if(elt->isA("Field")) { 
      V4 << "Found field: " << ce2s(elt) << endl;
      Field* f = (Field*)elt;
      if(f->domain==space) { return f;
      } if(f->domain->child_of(space)) {
        ierror(s,"Direct reference to child space in parent:"+ce2s(s));
      } else { // implicit restriction
        OI *oi = new OperatorInstance(s,Env::core_op("restrict"),space);
        oi->add_input(f);
        if(space->selector) oi->add_input(space->selector); 
        return oi->output;
      }
    } else if(elt->isA("Operator")) {
      V4 << "Lambda literal: " << ce2s(elt) << endl;
      return dfg->add_literal(new ProtoLambda((Operator*)elt),space,s);
    } else if(elt->isA("MacroSymbol")) {
      V4 << "Macro: " << ce2s(elt) << endl;
      return sexp_to_graph(((MacroSymbol*)elt)->pattern,space,env);
    } else return field_err(s,space,"Can't interpret "+elt->type_of()+" "+
                            s->to_str()+" as field");
  } else if(s->isScalar()) { // Numbers are literals
    V4 << "Numeric literal.\n";
    return dfg->add_literal(new ProtoScalar(((SE_Scalar*)s)->value),space,s);
  } else { // it must be a list
    // Lists are special forms or function applicatios
    SE_List* sl = (SE_List*)s;
    if(sl->len()==0) return field_err(sl,space,"Expression has no members"); 
    if(sl->op()->isSymbol()) { 
      // check if it's a special form
      string opname = ((SE_Symbol*)sl->op())->name;
      if(opname=="let") { return let_to_graph(sl,space,env,false);
      } else if(opname=="let*") { return let_to_graph(sl,space,env,true);
      } else if(opname=="all") { // evaluate children, returning last field
        Field* last=NULL;
        V4 << "Found 'all' construct\n";
        for(int j=1;j<sl->len();j++) last = sexp_to_graph((*sl)[j],space,env);
        return last;
      } else if(opname=="restrict"){ 
        return restrict_to_graph(sl,space,env);
      } else if(opname=="def" && sl->len()==3) { // variable definition
        SExpr *def=(*sl)[1], *exp=(*sl)[2];
        if(!def->isSymbol())
          return field_err(sl,space,"def name not a symbol: "+def->to_str());
        Field* f = sexp_to_graph(exp,space,env);
        env->force_bind(((SE_Symbol*)def)->name,f);
        V4 << "Defined variable: " << ce2s(f) << endl;
        return f;
      } else if(opname=="def" || opname=="primitive" || 
                opname=="lambda" || opname=="fun") {
        Operator* op = sexp_to_op(s,env);
        if(!(opname=="lambda" || opname=="fun")) return NULL;
        return dfg->add_literal(new ProtoLambda(op),space,s);
      } else if(opname=="annotate") {
        SE_List_iter li(sl); li.get_next(); // make iterator, discard op
        string name = li.get_token("operator name");
        CE* p = env->lookup(name);
        if(p==NULL) {
          compile_error(sl,"Can't find primitve '"+name+"' to annotate");
        } else if(!p->isA("Primitive")) {
          compile_error(sl,"Can't annotate '"+name+"': not a primitive");
        } else {
          parse_primitive_attributes(&li,(Primitive*)p); // add in attributes
        }
        return NULL; // annotations are like primitives: nothing returned
      } else if(opname=="letfed" || opname=="letfed+") {
        return letfed_to_graph(sl,space,env,opname=="letfed+");
      } else if(opname=="macro") {
        V4 << "Defining macro\n";
        sexp_to_macro(sl,env);
        return NULL;
      } else if(opname=="include") {
        for(int j=1;j<sl->len();j++) {
          SExpr *ex = (*sl)[j];
          V4 << "Including file: "<<ce2s(ex)<<endl;
          if(ex->isSymbol()) interpret_file(((SE_Symbol*)ex)->name);
          else compile_error(ex,"File name "+ex->to_str()+" is not a symbol");
        }
        return NULL;
      } else if(opname=="quote") {
        if(sl->len()!=2) 
          return field_err(sl,space,"Quote requires an argument: "+s->to_str());
        V4 << "Creating quote literal\n";
        return dfg->add_literal(quote_to_literal_type((*sl)[1]),space,s);
      } else if(opname=="quasiquote") {
        return field_err(sl,space,"Quasiquote only allowed in macros: "+sl->to_str());
      }
      // check if it's a macro
      CompilationElement* ce = env->lookup(opname);
      if(ce && ce->isA("Macro")) {
        V4 << "Applying macro\n";
        SExpr* new_expr;
        if(ce->isA("MacroOperator")) {
          new_expr = expand_macro((MacroOperator*)ce,sl);
          if(new_expr->attributes.count("DUMMY")) // Mark of a failure
            return field_err(s,space,"Macro expansion failed on "+s->to_str());
        } else { // it's a MacroSymbol
          new_expr = sl->copy();
          ((SE_List*)new_expr)->children[0]=((Macro*)ce)->pattern;
        }
        return sexp_to_graph(new_expr,space,env);
      }
    }
    // if we didn't return yet, it's an ordinary composite expression
    Operator *op = sexp_to_op(sl->op(),env);
    if(op->marked(":protected"))
      compile_warn(op,"operator '"+op->name+"' not intended for direct use.");
    OperatorInstance *oi = new OperatorInstance(s,op,space);
    for(vector<SExpr*>::iterator it=sl->args(); it!=sl->children.end(); it++) {
      Field* sub = sexp_to_graph(*it,space,env);
      // operator defs, primitives, and macros return null & are ignored
      if(sub) oi->add_input(sub);
    }
    if(!op->signature->legal_length(oi->inputs.size())) {
      compile_error(s,"Called "+ce2s(op)+" with "+i2s(oi->inputs.size())+
                    " arguments; it requires "+op->signature->num_arg_str());
    }
    V4 << "Added operator "<<ce2s(oi)<<endl;
    return oi->output;
  }
  ierror("Fell through sexp_to_graph w/o returning for: "+s->to_str());
}


// TOPLEVEL INTERPRETER CALLS
// Interpret acts by updating the contents of the DFG
void ProtoInterpreter::interpret(SExpr* sexpr, bool recursed) {
  // interpret the expression
  dfg->output = sexp_to_graph(sexpr,allspace,toplevel);
  terminate_on_error();
  // if finishing, designate relevant sections of the DFG
  if(!recursed) dfg->determine_relevant();
}

void ProtoInterpreter::interpret_file(string name) {
  ifstream* filestream = parent->proto_path.find_in_path(name);
  if(filestream==NULL)
    { compile_error("Can't find file '"+name+"'"); terminate_on_error(); }
  SExpr* sexpr= read_sexpr(name,filestream);
  compiler_error|=!sexpr; terminate_on_error();
  interpret(sexpr,true);
}

/*****************************************************************************
 *  EXTERNAL INTERFACE                                                       *
 *****************************************************************************/

ProtoInterpreter::ProtoInterpreter(NeoCompiler* parent, Args* args) {
  this->parent=parent;
  verbosity=args->extract_switch("--interpreter-initialization-verbosity") ? 
    args->pop_int() : 0;

  // initialize compiler variables
  toplevel = new Env(this); dfg = new DFG();
  dfg->attributes["CONTEXT"]=new Context("root",0);
  allspace = new AM(dfg,dfg);
  
  // load operators needed by interpreter
  V1 << "Loading bootstrap Proto operators...\n";
  interpret_file("bootstrap.proto"); Env::record_core_ops(toplevel);
  // load rest of operators
  populate_specials(); // can't shadow operators w. special syntactic handling
  V1 << "Loading core Proto operators...\n";
  interpret_file("core.proto"); Env::record_core_ops(toplevel);

  verbosity=args->extract_switch("--interpreter-verbosity") ? 
    args->pop_int() : parent->verbosity;
}

ProtoInterpreter::~ProtoInterpreter() {
  delete toplevel; delete allspace;
}

void ProtoInterpreter::interpret(SExpr* sexpr) { interpret(sexpr,false); }
