/* Expression evaluation node
 * Author: iiif
 * License: GPL-2.0-or-later */

#include "node_function_util.hh"
#include "UI_interface.hh"
#include "UI_resources.hh"
#include "exprtk.hpp"

namespace blender::nodes::node_fn_expr_eval_cc {

NODE_STORAGE_FUNCS(NodeExprEval)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>("a");
  b.add_input<decl::Float>("b");
  b.add_input<decl::Float>("c");
  b.add_input<decl::Float>("d");
  b.add_output<decl::Float>("result");
}

static void node_init(bNodeTree*, bNode* node)
{
  node->storage = MEM_callocN(sizeof(NodeExprEval), __func__);
}

static void node_storage_free(bNode* node)
{
  NodeExprEval* storage = (NodeExprEval*)node->storage;
  if (storage == nullptr)
    return;
  if (storage->expression != nullptr)
    MEM_freeN(storage->expression);
  MEM_freeN(storage);
}

static void node_storage_copy(bNodeTree*, bNode* dst, const bNode* src)
{
  auto* storage = (NodeExprEval*)src->storage;
  auto* cpy = (NodeExprEval*)MEM_dupallocN(storage);

  if (storage->expression) {
    cpy->expression = (char*)MEM_dupallocN(storage->expression);
  }

  dst->storage = cpy;
}

static void node_layout(uiLayout *layout, bContext*, PointerRNA *ptr)
{
  //uiLayout *col = uiLayoutColumn(layout, true);
  uiItemR(layout, ptr, "expression", UI_ITEM_NONE, "", ICON_NONE);
}

class ExprtkEvaluator : public mf::MultiFunction
{
public:
  using Type = float;
  
private:
  exprtk::expression<Type>   expr_;
  exprtk::symbol_table<Type> vars_;
  bool is_valid_ = false;
  mutable Type a = 0, b = 0, c = 0, d = 0;

public:
  ExprtkEvaluator(char const* expr_str)
  {
    vars_.add_constants();
    vars_.add_variable("a", a);
    vars_.add_variable("b", b);
    vars_.add_variable("c", c);
    vars_.add_variable("d", d);
    expr_.register_symbol_table(vars_);

    exprtk::parser<Type> parser;
    if (expr_str)
      is_valid_ = parser.compile(expr_str, expr_);
    else
      is_valid_ = false;

    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Expression Evaluator", signature};
      builder.single_input<Type>("a");
      builder.single_input<Type>("b");
      builder.single_input<Type>("c");
      builder.single_input<Type>("d");
      builder.single_output<Type>("result");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &input_as = params.readonly_single_input<float>(0, "a");
    const VArray<float> &input_bs = params.readonly_single_input<float>(1, "b");
    const VArray<float> &input_cs = params.readonly_single_input<float>(2, "c");
    const VArray<float> &input_ds = params.readonly_single_input<float>(3, "d");

    auto results = params.uninitialized_single_output<float>(4, "result");

    mask.foreach_index([&](const int i) {
      a = input_as[i];
      b = input_bs[i];
      c = input_cs[i];
      d = input_ds[i];
      results[i] = expr_.value();
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.min_grain_size = 65535;
    return hints;
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &node = builder.node();
  //static auto fn = mf::build::SI4_SO<float, float, float, float, float>("Eval Expr", [](float a, float b, float c, float d){
  //    return a+b;
  //  });
  //builder.set_matching_fn(fn);
  builder.construct_and_set_matching_fn<ExprtkEvaluator>(node_storage(node).expression);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_EXPR_EVALUATE, "Expression", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  blender::bke::node_type_storage(&ntype, "NodeExprEval", node_storage_free, node_storage_copy);
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register);

} // namespace blender::nodes::node_fn_expr_eval_cc
