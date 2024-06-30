/* Expression evaluation node
 * Author: iiif
 * License: GPL-2.0-or-later */

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_node_extra_info.hh"

#include "node_function_util.hh"

#define exprtk_disable_rtl_vecops
#define exprtk_disable_rtl_io
#define exprtk_disable_rtl_io_file
#include "exprtk.hpp"

#include <memory>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

namespace blender::nodes::node_fn_eval_expr_cc {

NODE_STORAGE_FUNCS(NodeEvalExpression)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>("A");
  b.add_input<decl::Float>("B");
  b.add_input<decl::Float>("C");
  b.add_input<decl::Float>("D");
  b.add_output<decl::Float>("Result");
}

static void node_init(bNodeTree *, bNode *node)
{
  node->storage = MEM_callocN(sizeof(NodeEvalExpression), __func__);
}

static void node_storage_free(bNode *node)
{
  NodeEvalExpression *storage = (NodeEvalExpression *)node->storage;
  if (storage == nullptr)
    return;
  if (storage->expression != nullptr)
    MEM_freeN(storage->expression);
  MEM_freeN(storage);
}

static void node_storage_copy(bNodeTree *, bNode *dst, const bNode *src)
{
  auto *storage = (NodeEvalExpression *)src->storage;
  auto *cpy = (NodeEvalExpression *)MEM_dupallocN(storage);

  if (storage->expression) {
    cpy->expression = (char *)MEM_dupallocN(storage->expression);
  }

  dst->storage = cpy;
}

static void node_layout(uiLayout *layout, bContext *, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "expression", UI_ITEM_NONE, "", ICON_NONE);
}

class MF_ExprtkEvaluator : public mf::MultiFunction {
 public:
  using ValueType = float;

 private:
  std::string expression_ = "";

  struct ThreadLocalEvaluator {
    exprtk::expression<ValueType> expr;
    exprtk::symbol_table<ValueType> vars;
    ValueType a, b, c, d;
  };
  mutable bool is_valid_ = true;
  mutable std::unordered_map<std::thread::id, std::unique_ptr<ThreadLocalEvaluator>> evaluators_;
  mutable std::shared_mutex lock_;

  ThreadLocalEvaluator &getThreadLocalEvaluator() const
  {
    lock_.lock_shared();
    auto existing_evaluator_iterator = evaluators_.find(std::this_thread::get_id());
    if (existing_evaluator_iterator != evaluators_.end()) {
      lock_.unlock_shared();
      return *existing_evaluator_iterator->second;
    }
    lock_.unlock_shared();

    auto evaluator = std::make_unique<ThreadLocalEvaluator>();
    auto &ref = *evaluator;
    if (is_valid_) {
      ref.vars.add_constants();
      // exprtk is case-insensitive, `A` is the same variable as `a`
      ref.vars.add_variable("a", ref.a);
      ref.vars.add_variable("b", ref.b);
      ref.vars.add_variable("c", ref.c);
      ref.vars.add_variable("d", ref.d);
      ref.expr.register_symbol_table(ref.vars);
      exprtk::parser<ValueType> parser;
      is_valid_ = parser.compile(expression_, ref.expr);
    }

    lock_.lock();
    evaluators_[std::this_thread::get_id()] = std::move(evaluator);
    lock_.unlock();
    return ref;
  }

 public:
  MF_ExprtkEvaluator(char const *expr_str)
  {
    if (expr_str)
      expression_ = expr_str;
    else
      expression_ = "";

    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Expression Evaluator", signature};
      builder.single_input<ValueType>("A");
      builder.single_input<ValueType>("B");
      builder.single_input<ValueType>("C");
      builder.single_input<ValueType>("D");
      builder.single_output<ValueType>("Result");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &input_as = params.readonly_single_input<float>(0, "A");
    const VArray<float> &input_bs = params.readonly_single_input<float>(1, "B");
    const VArray<float> &input_cs = params.readonly_single_input<float>(2, "C");
    const VArray<float> &input_ds = params.readonly_single_input<float>(3, "D");

    auto results = params.uninitialized_single_output<float>(4, "Result");
    auto &evaluator = getThreadLocalEvaluator();

    if (is_valid_) {
      mask.foreach_index([&](const int i) {
        evaluator.a = input_as[i];
        evaluator.b = input_bs[i];
        evaluator.c = input_cs[i];
        evaluator.d = input_ds[i];
        results[i] = evaluator.expr.value();
      });
    }
    else {
      mask.foreach_index([&](const int i) {
        results[i] = 0;
      });
    }
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.min_grain_size = 65535;
    return hints;
  }
};

static bool test_compile(char const* expression, std::string* out_error)
{
  exprtk::expression<float> expr;
  exprtk::symbol_table<float> vars;
  float a, b, c, d;

  vars.add_constants();
  vars.add_variable("a", a);
  vars.add_variable("b", b);
  vars.add_variable("c", c);
  vars.add_variable("d", d);
  expr.register_symbol_table(vars);
  exprtk::parser<float> parser;
  bool valid = parser.compile(expression, expr);
  if (valid && out_error)
    *out_error = parser.error();
  return valid;
}

static void node_extra_info(NodeExtraInfoParams &params)
{
  char const* expression = node_storage(params.node).expression;
  std::string error_message;
  if (!expression || !test_compile(expression, &error_message)) {
    NodeExtraInfoRow row;
    row.text = RPT_("Invalid Expression");
    row.tooltip = TIP_("Cannot evaluate this, result will be 0");
    row.icon = ICON_ERROR;
    params.rows.append(std::move(row));
  }
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &node = builder.node();
  char const* expression = node_storage(node).expression;
  builder.construct_and_set_matching_fn<MF_ExprtkEvaluator>(expression);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_EVAL_EXPRESSION, "Expression", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  blender::bke::node_type_storage(
      &ntype, "NodeEvalExpression", node_storage_free, node_storage_copy);
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  ntype.get_extra_info = node_extra_info;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register);

}  // namespace blender::nodes::node_fn_eval_expr_cc
