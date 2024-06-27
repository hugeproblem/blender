/* Expression evaluation node
 * Author: iiif
 * License: GPL-2.0-or-later */

#include "node_function_util.hh"
#include "UI_interface.hh"
#include "UI_resources.hh"

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

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &node = builder.node();
  static auto fn = mf::build::SI4_SO<float, float, float, float, float>("Eval Expr", [](float a, float b, float c, float d){
      return a+b;
    });
  builder.set_matching_fn(fn);
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
